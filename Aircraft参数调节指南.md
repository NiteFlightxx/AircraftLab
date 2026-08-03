# Aircraft 参数调节指南

本文说明 AircraftLab 当前代码中会影响飞行、自动驾驶、Root Motion 和模拟 LOD 的参数。默认值均指 C++ 新建资产时的默认值；项目中的 Data Asset、蓝图和物理资产可以覆盖它们。

## 1. 参数分层

调参前先区分参数归属：

| 层级 | 配置位置 | 负责内容 |
|---|---|---|
| 刚体 | Skeletal Mesh 的 Physics Asset / BodyInstance | 质量、质心、惯量、线性阻尼、角阻尼、碰撞 |
| 旋翼型号 | `UAirscrewProfileAsset` | 最大推力、反扭矩、电机响应、推力轴 |
| 旋翼实例 | `UAirscrewComponent` | 位置、名称、CW/CCW、启用状态 |
| 飞控 | `UFlightControllerProfileAsset` | 硬限制、PID、阻尼补偿、控制分配、替代驱动、Failure Policy |
| 自动驾驶机型策略 | `UAutopilotProfileAsset` | 路径制导、协调转弯、悬停推力估计 |
| 单次任务 | `Submit*` Command | 本次速度、加减速、Jerk、航向、到达判据、超时 |
| 性能策略 | `UAircraftSimulationLODProfileAsset` | 距离、驱动模式、Tick 间隔、碰撞、网络预算 |
| 玩家输入 | `UAircraftInputComponent` | IMC、IA 和飞控输入死区 |

优先级原则：

```text
单次任务软限制
  ≤ 飞控硬限制
  ≤ 机体和旋翼真实能力
```

如果上层要求超过下层能力，代码会限幅，但表现通常是跟踪滞后、轨迹切角、到点超调或长期饱和。

## 2. 调参前的基线

每个机型至少记录以下信息：

- 物理质量和质心；
- 三轴惯量；
- 线性阻尼和角阻尼；
- 旋翼数量、位置、推力轴和旋向；
- 单桨最大推力与反扭矩系数；
- FlightController、Airscrew、Autopilot 和 Simulation LOD Profile 版本；
- Chaos 固定物理步长；
- 测试时使用的飞行模式、命令参数和载荷。

不要把另一套质量、惯量或旋翼布局上的 PID 数值直接复制到新机型。

建议调试环境：

1. 暂时固定为 `FlightController` 驱动。
2. 暂时关闭 Autopilot，先验证手动稳定。
3. 使用无风、无外部约束、稳定帧率的测试关卡。
4. 一次只改变一个参数组。
5. 先从内环到外环，再调自动驾驶。

## 3. 刚体和物理资产

### 3.1 质量

质量直接决定相同推力下的线加速度：

```text
acceleration = force / mass
```

调大质量：

- 悬停所需总距增大；
- 升降和水平加速变慢；
- 旋翼更容易接近满推力；
- 原有速度和高度 PID 可能出现持续误差或积分累积。

调小质量则相反，过小会让相同控制输出过于猛烈。

悬停的最低物理条件：

```text
所有可用旋翼最大垂直推力之和 > mass × gravity
```

实际应保留姿态、转弯、抗扰和故障余量，不能只做到推重比刚好为 1。

### 3.2 质心

飞控使用 Chaos 的真实质心计算每个旋翼力臂。质心偏移会改变 Roll/Pitch 权限和悬停时各电机负载。

检查顺序：

1. 物理资产质心是否与预期一致；
2. 旋翼组件位置是否相对根 Mesh 正确；
3. 对称机体在悬停时，对称旋翼指令是否接近；
4. 控制分配是否存在某个旋翼长期饱和。

如果质心确实偏置，应保留真实偏置并重新调参，不要用错误旋翼位置抵消。

### 3.3 惯量

惯量越大，同一力矩产生的角加速度越小：

- Roll/Pitch/Yaw 响应变慢；
- 角速度环需要更大输出；
- 参考模型和最大角速度可能需要降低；
- 角阻尼补偿所需力矩增大。

惯量过小会使姿态非常敏感，更容易被较大的角速度 PID 增益激发振荡。

### 3.4 线性阻尼

Chaos 线性阻尼会产生近似与速度成比例的减速。飞控读取真实阻尼并进行前馈补偿，同时按阻尼计算可达巡航速度：

```text
damping_limited_speed
  = physical_horizontal_acceleration
  × (1 - reserve_fraction)
  / linear_damping
```

线性阻尼调大：

- 松开控制后自然减速更强；
- 恒速需要更大的持续倾角和推力；
- Autopilot 可达最大速度可能自动降低；
- 过大时即使提高 CruiseSpeed 也无法达到目标速度。

线性阻尼调小：

- 滑行距离变长；
- 对速度环和提前制动依赖更强；
- 太小且 PID 阻尼不足时容易超调和来回摆动。

推荐做法是使用满足游戏手感的中等物理阻尼，再用飞控阻尼前馈补偿恒速需求；不要仅靠极大阻尼实现稳定。

### 3.5 角阻尼

角阻尼调大：

- 旋转自然衰减更强；
- 达到指定角速度需要更多持续力矩；
- 偏航和压坡可能显得迟钝；
- 角阻尼前馈依赖更强。

角阻尼调小：

- 旋转更灵活；
- 角速度环 Kd 和参考模型需要承担更多阻尼；
- 过小时可能持续摇摆。

飞控用真实惯量、真实角阻尼和当前分配权限计算补偿，因此改变角阻尼后必须复测三轴角速度环。

## 4. Airscrew Profile

### 4.1 当前模型公式

对归一化命令 `c`：

```text
shaped_command = c ^ CommandExponent
target_rpm = lerp(IdleRpm, MaxRpm, shaped_command)  // c > 0
thrust = MaxThrustForce
       × Efficiency
       × ThrustCoefficient
       × (current_rpm / MaxRpm)²
reaction_torque = thrust
                × ReactionTorqueCoefficient
                × Efficiency
```

命令为 0 时目标 RPM 为 0。SpinUp/SpinDown 时间常数和命令 Slew 共同决定推力响应速度。

### 4.2 `FAircraftRotorDefinition`

| 参数 | C++ 默认值 | 实际作用 | 调大后的影响 |
|---|---:|---|---|
| `ThrustAxisLocal` | `(0,0,1)` | 机体局部推力方向 | 改方向会同时改变升力和各轴力矩；必须归一化且不能为零 |
| `MaxThrustForce` | 245 N | 单桨最大静推力基值 | 推力、总距权限和姿态权限增大 |
| `ThrustCoefficient` | 2.0 | 推力整体乘数 | 与 MaxThrustForce 等价地放大物理推力 |
| `ReactionTorqueCoefficient` | 100 m | `Torque = Thrust × Coefficient` | 偏航权限急剧增大；必须按机型标定 |
| `Efficiency` | 1.0 | 推力和反扭矩有效度 | 降低会减少推力；当前反扭矩还会再次乘 Efficiency |
| `ControlAuthorityScale` | 1.0 | 分配器允许使用的最大推力比例 | 降低会主动降额该型号旋翼，不改变 Profile 的物理最大推力定义 |
| `CommandScale` | 1.0 | 分配结果到电机命令的统一缩放 | 增大到 1 以上也会在最终命令处截到 1；通常保持 1 |
| `Motor` | 见下表 | 电机动态 | 影响推力建立与消退速度 |

`ReactionTorqueCoefficient` 的量纲是米，不是无量纲值。C++ 默认值只是结构默认值，实际机型应使用经过验证的 Data Asset 数值。

### 4.3 `FAircraftMotorModelConfig`

| 参数 | 默认值 | 调大后的影响 | 典型风险 |
|---|---:|---|---|
| `IdleRpm` | 1500 RPM | 非零命令时更快建立基础推力 | 最小非零推力升高，低总距控制变粗 |
| `MaxRpm` | 12000 RPM | 只改变 RPM 标尺；最大推力仍由推力参数定义 | 与动画/音效或实测曲线不一致 |
| `SpinUpTimeSeconds` | 0.06 s | 加速更慢 | 姿态和高度响应滞后、超调 |
| `SpinDownTimeSeconds` | 0.10 s | 减速更慢 | 到点和姿态回正时推力消退慢 |
| `CommandExponent` | 2.0 | 低命令区更弱，高命令区更集中 | 悬停附近分辨率不足 |
| `MaxCommandSlewPerSecond` | 8.0 | 指令允许变化更快 | 太大产生尖锐推力变化；太小让控制器追不上 |

`MaxCommandSlewPerSecond = 0` 表示不限制命令变化率，不是停用电机。

### 4.4 旋翼实例参数

| 参数 | 作用 | 要求 |
|---|---|---|
| `RotorProfile` | 共享旋翼型号 | 必须有效 |
| `RotorName` | 故障寻址和健康映射 | 每个旋翼唯一且稳定 |
| `SpinDirection` | 反扭矩符号 | CW/CCW 按布局交替，悬停偏航力矩应近似抵消 |
| `bRotorEnabled` | 是否参与模拟和分配 | 关闭后会刷新飞控引用 |
| `bApplyForce` | 是否实际向 Chaos 施力 | 主要用于调试，不等于健康故障接口 |

旋翼位置来自组件 Transform，相对 Actor 根 Mesh 计算。推力轴来自 Profile，不读取 Airscrew 组件旋转。

### 4.5 旋翼标定顺序

1. 固定质量、质心和旋翼位置。
2. 设置正确推力轴和旋向。
3. 根据实测或目标推重比标定 `MaxThrustForce × ThrustCoefficient × Efficiency`。
4. 调 `ReactionTorqueCoefficient`，使偏航权限足够但不压倒 Roll/Pitch。
5. 验证对称悬停的电机指令。
6. 再调 SpinUp、SpinDown、Exponent 和 Slew。
7. 最后调飞控 PID。

## 5. FlightController Profile

### 5.1 Input

| 参数 | 默认值 | 调大后的影响 |
|---|---:|---|
| `HorizontalHoldStickDeadband` | 0.08 | 更容易识别为松杆并进入水平制动/保持，小输入被忽略 |
| `VerticalHoldStickDeadband` | 0.08 | 更容易保持高度，小幅升降输入被忽略 |
| `YawHoldStickDeadband` | 0.05 | 更容易锁航向，小幅偏航输入被忽略 |
| `HorizontalBrakeToHoldSpeedCmPerSec` | 20 cm/s | 更早锁定停止位置；过大时还未充分减速就锁点 |

水平 Roll/Pitch 只判断是否超过死区，超过后仍按原始输入幅度映射最大速度；垂直 Throttle 会去除死区后重新映射到完整 `[0,1]` 幅度。

### 5.2 Execution

| 参数 | 默认值 | 作用 |
|---|---:|---|
| `bControllerEnabledByDefault` | true | 决定飞控是否开始 Tick；关闭会停止旋翼输出 |

`AAircraftPawn` 仍会在 BeginPlay 调用 Arm，但 Controller Disabled 时不会运行控制循环。

### 5.3 Body Axes

| 参数 | 默认值 | 作用 |
|---|---|---|
| `ForwardAxis` | `PositiveY` | 模型真实机头方向，可选 `+X/+Y/-X/-Y` |

判断错误的常见表现：

- 前进指令变成侧移；
- Roll/Pitch 互换或符号错误；
- FaceVelocity/FaceTarget 偏 90° 或 180°；
- Root Motion 旋转和飞控航向不一致。

先修正 `ForwardAxis`，不要通过交换输入轴、旋翼位置或 PID 符号补偿。

### 5.4 Limits

新建 FlightController Profile 的有效默认值由 `FlightControllerConfig::InitializeDefaults` 设置：

| 参数 | 默认值 | 调大后的影响 | 主要限制 |
|---|---:|---|---|
| `MaxTiltAngleDegrees` | 25° | 最大水平加速度和转弯能力增加 | 垂直升力余量下降 |
| `MaxYawRateDegreesPerSec` | 90°/s | 最大转头速度增加 | 偏航权限和角阻尼 |
| `MaxRollRateDegreesPerSec` | 180°/s | Roll 建立更快 | Roll 角速度环和力矩权限 |
| `MaxPitchRateDegreesPerSec` | 180°/s | Pitch 建立更快 | Pitch 角速度环和力矩权限 |
| `MaxClimbRateCmPerSec` | 300 | 最大上升速度增加 | 推重比和垂直阻尼 |
| `MaxDescentRateCmPerSec` | 200 | 最大下降速度增加 | 最小总距和停止高度 |
| `MaxHorizontalSpeedCmPerSec` | 800 | 手动和 Autopilot 水平硬上限增加 | 阻尼可达速度 |
| `MaxHorizontalAccelerationCmPerSecSq` | 600 | 速度变化更快 | 同时受 `g × tan(MaxTilt)` 限制 |
| `MaxVerticalAccelerationCmPerSecSq` | 500 | 升降设定变化更快 | 推力余量 |
| `MinCollectiveCommand` | 0.0 | 最小总距提高 | 下降和快速卸载能力下降 |
| `HoverCollectiveCommand` | 0.50 | 手动和估计器初始悬停基准提高 | 过高上升，过低下降 |
| `MaxCollectiveCommand` | 1.0 | 可用最大总距提高 | 旋翼饱和和物理极限 |

必须满足：

```text
MinCollectiveCommand
  ≤ HoverCollectiveCommand
  ≤ MaxCollectiveCommand
```

物理可用水平加速度为：

```text
min(
  MaxHorizontalAcceleration,
  gravity × tan(MaxTiltAngle)
)
```

因此只提高水平加速度而不提高倾角，可能没有任何效果。

### 5.5 PID 字段

`FAircraftPidGains`：

| 字段 | 作用 | 调大后的常见表现 |
|---|---|---|
| `Kp` | 当前误差的即时修正 | 更紧、更快；过大振荡 |
| `Ki` | 消除持续静差 | 抗恒定扰动更强；过大产生慢摆和回弹 |
| `Kd` | 抑制测量变化 | 阻尼增加；过大噪声、发涩 |
| `Kff` | 外部设定值前馈 | 减少跟踪滞后；过大超前或饱和 |
| `IntegralLimit` | 积分状态绝对值上限 | 可补偿更大恒定误差；不是最终输出限幅 |
| `OutputLimit` | 本环输出绝对值上限 | 下一级权限增加；过大可能超出物理能力 |
| `DerivativeCutoffHz` | D 项低通截止频率，0 为不滤波 | 越高跟随快但噪声多 |
| `bFreezeIntegralWhenSaturated` | 输出饱和时回退本步积分 | 一般保持开启 |

`FAircraftFeedbackPidGains` 不暴露 `Kff`，用于没有外部前馈入口的角速度和垂直速度反馈环。

### 5.6 默认控制环参数

| 控制环 | 默认参数 | 输出 |
|---|---|---|
| Position X/Y | `Kp 0.40, Ki 0, Kd 0.30, Kff 1.0, I 0, Out 800` | cm/s |
| Velocity X/Y | `Kp 1.50, Ki 0.01, Kd 0.60, Kff 1.0, I 3000, Out 600, D 12 Hz` | cm/s² |
| Quaternion Roll/Pitch/Yaw | `4.5 / 4.5 / 3.0` | degree/s |
| Rate Roll/Pitch | `Kp 0.0080, Ki 0.00100, Kd 0.00040, I 120, Out 0.35, D 18 Hz` | 归一化轴指令 |
| Rate Yaw | `Kp 0.0012, Ki 0.00015, Kd 0.00008, I 120, Out 0.20, D 15 Hz` | 归一化轴指令 |
| Altitude | `Kp 1.20, Ki 0, Kd 0.20, Kff 1.0, I 0, Out 300` | cm/s |
| Vertical Velocity | `Kp 0.0015, Ki 0.00020, Kd 0.00050, I 2500, Out 0.30, D 10 Hz` | 总距偏移 |

位置、高度和速度环使用 derivative-on-measurement 路径，减少设定值阶跃造成的微分冲击。

### 5.7 Attitude

| 参数 | 默认值 | 调大后的影响 |
|---|---:|---|
| `QuaternionAttitudeGains.Roll` | 4.5 | Roll 姿态误差更快转换成角速度 |
| `QuaternionAttitudeGains.Pitch` | 4.5 | Pitch 姿态误差更快转换成角速度 |
| `QuaternionAttitudeGains.Yaw` | 3.0 | 航向纠偏更快 |
| `AngularDampingFeedForwardScale` | 1.0 | 角阻尼补偿更强 |
| `bEnableAttitudeRefModel` | true | 平滑 Roll/Pitch 目标 |
| `RefModelNaturalFrequency` | 6 rad/s | 参考模型更快、更接近阶跃 |
| `RefModelRateFFLimitDegPerSec` | 100°/s | 允许更大的参考角速度前馈 |

推荐调节顺序：

1. 关闭或降低姿态外环激励，只调 Rate PID。
2. 分别测试 Roll、Pitch、Yaw 小阶跃。
3. Rate 环稳定后再调 Quaternion Gains。
4. 最后启用参考模型，调自然频率和前馈限幅。
5. 改变物理角阻尼后重新检查 `AngularDampingFeedForwardScale`。

- Rate 环过弱：角速度跟不上、姿态慢、积分逐渐建立。
- Rate 环过强：高频抖动、旋翼指令快速交替、分配残差增加。
- 姿态增益过强：角速度设定频繁触顶，姿态过冲。
- 参考模型频率过低：输入柔和但明显迟钝。
- 参考模型频率过高：接近角度阶跃，内环压力增大。

### 5.8 Position

| 参数 | 默认值 | 作用 |
|---|---:|---|
| `PositionGains.X/Y` | 见默认控制环表 | 位置误差 → 期望速度 |
| `VelocityGains.X/Y` | 见默认控制环表 | 速度误差 → 期望加速度 |
| `LinearDampingFeedForwardScale` | 1.0 | 补偿 Chaos 水平线性阻尼 |
| `DampingAccelerationReserveFraction` | 0.2 | 为转弯、抗扰和模型误差保留水平加速度 |

`LinearDampingFeedForwardScale`：

- 0：完全关闭补偿；
- 1：按 Chaos 阻尼模型完整补偿；
- 小于 1：保守补偿；
- 大于 1：过补偿，可能导致恒速超前。

`DampingAccelerationReserveFraction` 调大：

- 可达巡航速度降低；
- 留给转弯和抗扰的加速度增加；
- 高速路径更稳健但更慢。

位置超调先检查速度环和减速度，不要先加大 Position Kp。位置环过强会在越过目标后给出更大的反向速度，形成来回调整。

### 5.9 Altitude

| 参数 | 默认值 | 作用 |
|---|---:|---|
| `AltitudeGains` | 见默认控制环表 | 高度误差 → 垂直速度 |
| `VerticalVelocityGains` | 见默认控制环表 | 垂直速度误差 → 总距偏移 |
| `VerticalDampingFeedForwardScale` | 1.0 | 补偿垂直线性阻尼 |

调节顺序：

1. 校准 `HoverCollectiveCommand`。
2. 先固定垂直速度目标，调 Vertical Velocity 环。
3. 再调 Altitude 环。
4. 最后启用 Autopilot 悬停推力估计器。

悬停持续下沉或上升：

- 先查悬停总距；
- 再查推力配置和质量；
- 再查 Vertical Velocity Ki；
- Autopilot 下再查悬停推力估计值。

### 5.10 Allocator

| 参数 | 默认值 | 调大后的影响 |
|---|---:|---|
| `DampedPseudoInverseLambda` | 0.05 | 数值更稳定，但控制分配更软、残差更大 |
| `bEnableTiltCompensation` | true | 倾斜时增加总距，减少掉高 |
| `MinCosTilt` | 0.10 | 限制倾斜补偿最大倍率；越大越保守 |

`MinCosTilt` 必须在 `[0.05, 1]`。值过小会允许大倾角下出现非常高的总距请求，随后被最大总距截断；值过大则较早限制补偿，转弯容易掉高。

如果分配器长期饱和，优先检查：

- 旋翼位置和旋向；
- 推力与反扭矩系数；
- 质心；
- 最大倾角、角速度和 PID 输出；
- 是否有旋翼降效；
- Lambda 是否过大。

### 5.11 PhysicsConstraint 驱动

| 参数 | 默认值 | 调大后的影响 |
|---|---:|---|
| `LinearPositionStrength` | 100 | 更强地追位置目标 |
| `LinearVelocityStrength` | 20 | 更强地追速度并抑制线性振荡 |
| `LinearForceLimit` | 0 | 最大线性驱动力；0 传给 Constraint 表示不设置有限上限 |
| `AngularPositionStrength` | 100 | 更强地追旋转目标 |
| `AngularVelocityStrength` | 20 | 更强地追角速度并抑制旋转振荡 |
| `AngularTorqueLimit` | 0 | 最大驱动力矩；0 表示不设置有限上限 |
| `bAccelerationMode` | true | Drive 强度按加速度模式解释，质量变化下手感更一致 |

调节方法：

1. 先把位置/角度 Strength 设为较低值。
2. 增加 Velocity Strength 到不持续振荡。
3. 再提高 Position Strength 获得所需跟随精度。
4. 最后设置 Force/Torque Limit，避免远距离误差产生过大冲击。

位置 Strength 高、速度 Strength 低时容易弹簧振荡；速度 Strength 过高则运动迟钝。

### 5.12 Kinematic 驱动

| 参数 | 默认值 | 调大后的影响 |
|---|---:|---|
| `bSweepMovement` | true | 移动时做碰撞 Sweep |
| `PositionCorrectionRate` | 8 | 更快把速度预测位置纠正到目标位置 |
| `RotationInterpSpeed` | 8 | 更快跟随目标旋转 |

位置更新为“目标速度预测 + 指数位置纠偏”。Correction Rate 为 0 时只按目标速度积分，不向目标位置收敛。

### 5.13 Failure Policy

默认关闭。

| 参数 | 默认值 | 作用 |
|---|---:|---|
| `bEnabled` | false | 是否评估旋翼数量和控制权限 |
| `bEvaluateOnlyWhenArmed` | true | 仅 Armed 时评估 |
| `MinimumHealthyRotorCount` | 4 | 健康旋翼数下限；0 不检查 |
| `MinimumCollectiveAuthority` | 0.25 | 总距剩余权限下限；0 不检查 |
| `MinimumRollAuthority` | 0.25 | Roll 权限下限 |
| `MinimumPitchAuthority` | 0.25 | Pitch 权限下限 |
| `MinimumYawAuthority` | 0.25 | Yaw 权限下限 |
| `ConfirmationTimeSeconds` | 0.10 s | 故障持续多久才触发 |
| `RecoveryConfirmationTimeSeconds` | 1.0 s | 非锁存模式下健康多久才清除 |
| `bLatchTriggeredAction` | true | 是否保持触发状态直到显式重置 |
| `Action` | WarningOnly | Warning、切模式、Failsafe、EmergencyStop |
| `DegradedFlightMode` | Angle | `SwitchFlightMode` 的目标模式 |

权限值是相对全健康旋翼布局的比例。阈值越高越保守，也越容易在部分降效时触发。

启用 Failsafe 或 EmergencyStop 前必须测试：

- 停桨对 Gameplay 和网络的影响；
- 锁存后的恢复流程；
- `ResetFailurePolicyLatch` 的调用时机；
- 当前故障仍存在时重新触发的行为。

## 6. Autopilot 单次命令参数

### 6.1 有限轨迹约束

适用于 MoveTo、FollowPath、CircleArc 和 FlightController Root Motion：

| 参数 | 默认值 | 调大后的影响 |
|---|---:|---|
| `CruiseSpeedCmPerSec` | 800 | 名义巡航更快 |
| `MaxAccelerationCmPerSecSq` | 400 | 起步和追轨迹更快 |
| `MaxDecelerationCmPerSecSq` | 400 | 更晚也能停住、制动更强 |
| `MaxJerkCmPerSecCubed` | 2000 | 加速度建立更快、动作更硬 |
| `MaxClimbRateCmPerSec` | 300 | 上升更快 |
| `MaxDescentRateCmPerSec` | 200 | 下降更快 |
| `MaxVerticalAccelerationCmPerSecSq` | 500 | 垂直速度变化更快 |
| `MaxVerticalJerkCmPerSecCubed` | 1500 | 垂直加速度变化更快 |
| `MaxYawRateDegPerSec` | 90 | 转头更快 |
| `MaxYawAccelerationDegPerSecSq` | 180 | 偏航速度建立/制动更快 |
| `MaxYawJerkDegPerSecCubed` | 600 | 偏航加速度变化更快 |

所有 Movement Intent 都要求 `MaxAccelerationCmPerSecSq` 和 `MaxDecelerationCmPerSecSq` 严格大于 0；负的 Cruise Speed、Pass-through Speed、Jerk、到达容差、稳定时间或 Timeout 会使提交被拒绝。需要产生有限位移时，Cruise Speed 还应设为正数。

理论制动距离近似：

```text
braking_distance = (current_speed² - end_speed²)
                 / (2 × max_deceleration)
```

Jerk 限制会让实际制动力不能瞬间达到最大值，因此需要额外余量。高速到点超调时按顺序检查：

1. `MaxDeceleration` 是否足够；
2. Jerk 是否过小导致制动建立过慢；
3. 飞控硬加速度和最大倾角是否更低；
4. 线性阻尼可达限制是否压缩了轨迹权限；
5. 速度 PID 是否能跟踪负加速度；
6. 旋翼和电机 SpinDown 是否过慢。

### 6.2 持续运动约束

适用于 Velocity 和 Orbit：

| 参数 | 默认值 |
|---|---:|
| `MaxAccelerationCmPerSecSq` | 400 |
| `MaxJerkCmPerSecCubed` | 2000 |
| `MaxClimbRateCmPerSec` | 300 |
| `MaxDescentRateCmPerSec` | 200 |
| `MaxVerticalAccelerationCmPerSecSq` | 500 |
| `MaxVerticalJerkCmPerSecCubed` | 1500 |
| `MaxYawRateDegPerSec` | 90 |
| `MaxYawAccelerationDegPerSecSq` | 180 |
| `MaxYawJerkDegPerSecCubed` | 600 |

持续命令没有终点制动参数：

- Velocity 的巡航速度由 `DesiredVelocityCmPerSec` 决定；
- Orbit 的线速度由 `abs(radius × angular_rate)` 决定；
- 停止需要取消、替换命令或 Timeout。

此外，持续命令在应用时会把 `MaxDeceleration` 强制设为 `MaxAcceleration`，形成单一对称速率限制，不产生独立制动阶段。因此持续命令中只需调 `MaxAcceleration`。

### 6.3 到达判据

| 参数 | 默认值 | 调大后的影响 |
|---|---:|---|
| `HorizontalToleranceCm` | 50 | 更容易完成，停点精度降低 |
| `VerticalToleranceCm` | 100 | 更容易完成，高度精度降低 |
| `SpeedToleranceCmPerSec` | 50 | 允许带更大残余速度完成 |
| `YawToleranceDegrees` | 5° | 允许更大航向误差完成 |
| `StableTimeSeconds` | 0.2 s | 更抗瞬态穿越和抖动，完成延迟增加 |

到达判据不改变飞行控制本身，只决定何时把 Intent 标为成功。不能用放大容差掩盖持续振荡。

### 6.4 到达模式

`StopAndComplete`：

- 终点速度为 0；
- 满足完整到达判据后成功；
- 成功后在最终目标位置 Hold。

`PassThrough`：

- 终点保留 `PassThroughSpeedCmPerSec`；
- 轨迹完成后成功；
- 自动转为内部 Velocity 状态继续保持退出速度。

`PassThroughSpeed` 必须不大于 CruiseSpeed，实际会被截到 CruiseSpeed。

### 6.5 航向参数

| 参数 | 默认值 | 生效条件 |
|---|---:|---|
| `Mode` | FaceVelocity | 所有命令 |
| `FixedYawDegrees` | 0° | FixedYaw |
| `YawRateDegreesPerSec` | 90°/s | FixedYaw，并与 Motion Constraint 最大 Yaw Rate 取最小 |
| `bUseLookAtTarget` | false | FaceTarget |
| `LookAtPositionCm` | `(0,0,0)` | 独立世界点，或 Actor 相对偏移 |
| `LookAtActor` | null | 独立动态目标 |

FixedYaw 会根据剩余航向误差和最大角加速度计算制动限制，不会始终以满偏航速度冲到目标角。

FaceTarget 的选择：

- 有独立 LookAt：朝独立点/Actor；
- 无独立 LookAt：朝移动 Target；
- FollowPath 无独立 LookAt：朝最后一个路径点。

### 6.6 MoveTo

| 参数 | 含义 |
|---|---|
| `TargetPositionCm` | 无 Actor 时为世界目标；有 Actor 时为相对 Actor 的偏移 |
| `TargetActor` | 可选动态目标 |
| `Options` | 有限轨迹、航向、到达和超时 |

普通停点使用 `StopAndComplete`。连续多段巡逻可逐段提交，或在中间段使用 PassThrough。

### 6.7 FollowPath

| 参数 | 含义 |
|---|---|
| `PathPointsCm` | 至少两个世界空间点 |
| `TrajectoryMode` | PiecewiseLinear、Bezier、MinimumSnap |
| `Options` | 有限轨迹配置 |

模式选择：

- `PiecewiseLinear`：严格沿折线几何，适合外部导航点；相邻点不能重合；
- `Bezier`：点是控制点，中间点通常不经过；
- `MinimumSnap`：经过所有点并改变点间曲线和时间分配。

PiecewiseLinear 尖角没有独立的拐角速度参数。减小 CruiseSpeed、减小 Pure Pursuit 前瞻、提高可用加速度可以提高贴角能力，但过小前瞻或过高增益会引起蛇形。

### 6.8 Orbit

| 参数 | 默认值 | 作用 |
|---|---:|---|
| `CenterPositionCm` | 0 | 世界圆心或 Actor 偏移 |
| `CenterActor` | null | 动态圆心 |
| `RadiusCm` | 500 | 环绕半径，必须大于 0 |
| `AngularRateDegPerSec` | 45 | 正值逆时针，负值顺时针，不能为 0 |
| `TimeoutSeconds` | 0 | 0 表示持续到取消 |

Orbit 轨迹段自身还支持圈数限制（不在命令结构中，由轨迹段 `UOrbitTrajectorySegment` 持有）：

| 参数 | 默认值 | 作用 |
|---|---:|---|
| `bLoopLimitEnabled` | false | 是否启用圈数上限（默认关闭，无限盘旋） |
| `LoopCount` | 1 | 限定圈数（`bLoopLimitEnabled = true` 时有效） |

线速度：

```text
speed = radius × abs(radians(angular_rate))
```

半径不变时提高角速度，会按比例提高线速度和按平方提高向心加速度需求。实际可实现性受最大倾角和横向加速度限制。

### 6.9 CircleArc

| 参数 | 默认值 | 作用 |
|---|---:|---|
| `RadiusCm` | 500 | 水平圆弧半径 |
| `StartAngleDegrees` | 0° | 世界 +X 方向为 0° |
| `EndAngleDegrees` | 90° | 大于 Start 为逆时针，小于为顺时针 |

Start 与 End 不能相同。角度差可以超过 360°，表示多圈有限圆弧。调用方应保证机体接入圆弧起点的方式符合玩法需求。

### 6.10 Velocity

`DesiredVelocityCmPerSec` 是世界空间速度。XY 决定水平移动，Z 决定升降。它是持续命令，不会自行成功；使用取消、替换或 Timeout 结束。

## 7. Autopilot Profile

### 7.1 协调转弯

| 参数 | 默认值 | 调大后的影响 |
|---|---:|---|
| `bEnableCoordinatedTurns` | true | 开启路径方向变化的 Roll/YawRate 前馈 |
| `CoordinatedTurnSpeedThresholdCmPerSec` | 300 | 更高速度才进入压坡转弯 |
| `MaxBankAngleDegrees` | 35° | 允许更大协调 Roll |
| `MaxLateralAccelCmPerSecSq` | 500 | 允许更急的转弯 |

`MaxBankAngleDegrees` 最好不高于飞控 `MaxTiltAngleDegrees`，否则额外 Roll 最终仍会被飞控截断。

转弯掉高：

- 检查倾斜补偿；
- 降低 MaxBank 或 CruiseSpeed；
- 增加推力余量；
- 检查 MaxCollective 是否饱和。

### 7.2 路径制导

`GuidanceStrategy` 默认 `PurePursuit`。

Pure Pursuit：

| 参数 | 默认值 | 调大后的影响 |
|---|---:|---|
| `LookAheadGain` | 0.5 | 高速时看得更远，转向更平滑但更容易切角 |
| `MinLookAheadCm` | 100 | 低速更平滑，贴线精度下降 |
| `MaxLookAheadCm` | 1000 | 允许高速时更长的前瞻 |

前瞻距离为：

```text
clamp(
  LookAheadGain × horizontal_speed + MinLookAhead,
  MinLookAhead,
  MaxLookAhead
)
```

Vector Field：

| 参数 | 默认值 | 调大后的影响 |
|---|---:|---|
| `CrossTrackGain` | 0.01 | 横向回线更强；过大蛇形振荡 |
| `MaxCrossTrackCorrectionCm` | 500 | 允许更强的最大横向修正 |

Direct 没有额外参数，直接使用轨迹名义速度。

制导只应用于 FollowPath、Orbit 和 CircleArc；MoveTo 不经过 Pure Pursuit 或 Vector Field。

### 7.3 悬停推力估计器

| 参数 | 默认值 | 调大后的影响 |
|---|---:|---|
| `bEnableHoverThrustEstimator` | true | 在线修正 Autopilot 推力前馈基准 |
| `InitialStateVariance` | 0.01 | 初期更愿意快速改变估计 |
| `ProcessNoiseVariance` | `12.5e-6` | 更快跟踪载荷变化，但更噪 |
| `AccelNoiseVariance` | 5.0 | 更不相信加速度测量，估计更慢 |
| `GateSize` | 3.0 | 更少拒绝异常测量，机动更可能污染估计 |
| `MinHoverThrust` | 0.1 | 估计下限 |
| `MaxHoverThrust` | 0.9 | 估计上限 |

估计器只在 Autopilot Tick 中更新，并以 FlightController `HoverCollectiveCommand` 为初值。

调节建议：

1. 先让固定 HoverCollective 基本正确。
2. 再启用估计器。
3. 正常悬停时观察 `GetEstimatedHoverThrust`。
4. 估计抖动先提高 AccelNoise 或降低 ProcessNoise。
5. 载荷变化跟踪太慢则小幅提高 ProcessNoise。
6. 激烈机动污染估计时降低 GateSize。

### 7.4 Motion Profile

Motion Profile 的大部分限幅值（`FMotionProfileLimits`：最大速度、加速度、Jerk、偏航率等）不从 Profile 资产配置，而是每帧由 `ApplyIntentMotionLimits` 从当前 Intent 的 `FTrajectoryMotionConstraints` 与飞控硬限制取最小值重新推送。因此调运动限幅应改命令结构中的 `FTrajectoryMotionConstraints`，不是改 `UMotionProfile` 对象上的 `Limits`。

`UMotionProfile` 上有两个不被推送、保留自身默认值的独立增益：

| 参数 | 默认值 | 作用 |
|---|---:|---|
| `PositionCorrectionGain` | 1.5 | 位置误差→速度修正的比例增益（1/s）。越大收敛越快但越接近阶跃 |
| `PositionCorrectionFraction` | 0.5 | 位置闭合修正的最大速度占 `MaxHorizontalSpeed` 的比例 |

位置闭合修正仅在名义速度接近零时（停点/终态设定值）应用，用于把机体拉到精确目标位置。`PositionCorrectionGain` 过大会产生阶跃式修正和超调；过小则到点后残余偏差收敛慢。`PositionCorrectionFraction` 限制修正速度，避免末段冲过目标。

## 8. Montage 与 Root Motion 参数

### 8.1 播放参数

| 参数 | 默认值 | 影响 |
|---|---:|---|
| `Montage` | null | 必须是可播放 Montage |
| `PlayRate` | 1.0 | 同时改变动画和单位真实时间内的 Root Motion 速度 |
| `StartPositionSeconds` | 0 | 必须位于 `[0, MontageLength)` |
| `bStopAllMontages` | true | 播放前是否停止其他 Montage |

播放速率提高一倍时，同一 Root Motion 位移会在约一半时间内完成，因此所需速度、加速度和跟踪能力都会提高。

### 8.2 FlightController Root Motion

额外参数：

- `bApplyRootMotionRotation`：true 使用动画旋转；false 使用 Heading Options；
- `MotionConstraints`：限制从动画提取的目标运动；
- `ArrivalCriteria`：Montage 结束后判断飞控是否收敛；
- `TimeoutSeconds`：包含播放和播放后的收敛时间。

如果动画 Root Motion 速度超过 Motion Constraints，Motion Profile 会落后于动画累计目标；Montage 结束后 Intent 会继续等待飞控追到最终目标。

### 8.3 PhysicsConstraint Root Motion

使用 Root Motion 产生位置、速度、旋转和角速度目标，跟踪品质由 FlightController Profile 的 Constraint Simulation 参数决定。

如果出现弹簧振荡，优先增加 Velocity Strength 或降低 Position Strength；如果目标追不上，再逐步提高 Position Strength 和驱动力上限。

### 8.4 Kinematic Root Motion

跟踪品质由 PositionCorrectionRate、RotationInterpSpeed 和 Sweep 决定。它不使用 PID、旋翼推力或 Constraint 强度。

Sweep 命中障碍后，位置可能无法达到累计 Root Motion 最终目标，从而等待到达或最终超时。

### 8.5 `PlayMontage`

含 Root Motion 时自动使用当前 LOD DriveMode：

- FlightController：使用默认 Motion Constraints；
- PhysicsConstraint：使用当前 Constraint 参数；
- Kinematic：使用当前 Kinematic 参数；
- None：播放失败。

需要明确后端或专门参数时，使用对应的 `SubmitRootMotion*`，不要使用自动选择接口。

## 9. Simulation LOD Profile

### 9.1 全局评估参数

| 参数 | 默认值 | 调大后的影响 |
|---|---:|---|
| `EvaluationIntervalSeconds` | 0.25 s | 单机评估更少，切换响应更慢 |
| `MaxEvaluationsPerFrame` | 8 | 每帧可处理更多无人机，峰值开销增加 |
| `DistanceHysteresisCm` | 2000 | 边界抖动减少，进入/退出切换更迟 |
| `MinimumLODResidenceSeconds` | 1.0 s | 更少频繁切级，短时重要性变化响应受限 |
| `CombatKeepAliveSeconds` | 5.0 s | 战斗/受伤结束后更久保持最高优先级 |
| `bAuthoritySimulationOnly` | true | 只在服务器执行 NPC 模拟 |
| `bClientProxyUsesDefaultPhysicsReplication` | true | 物理 LOD 的客户端代理保持 Chaos 以接收物理复制 |

World Subsystem 每 0.1 秒刷新一次玩家位置，并在所有注册组件的 `MaxEvaluationsPerFrame` 中取最大值作为当帧全局评估预算。

### 9.2 每级 LOD 参数

| 参数 | 作用 | 注意 |
|---|---|---|
| `Name` | 调试名称 | 不参与选择 |
| `DriveMode` | 当前运动后端 | 数组可自由组合，不由索引写死 |
| `MaxDistanceCm` | 到最近玩家的上限 | 最后一个数组项忽略此值 |
| `bRunSlowLogic` | 是否允许 Autopilot Tick | 当前不控制飞控物理 Tick |
| `SlowLogicIntervalSeconds` | Autopilot Tick 间隔 | 0 为每游戏帧 |
| `CollisionMode` | Disabled/QueryOnly/QueryAndPhysics | 应与 DriveMode 和玩法需求匹配 |
| `SuggestedNetUpdateFrequency` | Authority Actor 网络频率 | 最低按 1 Hz 处理 |
| `bAllowDebugDraw` | 允许旋翼调试绘制 | Airscrew 自身 Debug 也必须开启 |
| `bEnableNetworkDormancy` | 进入网络休眠 | 切换时会 ForceNetUpdate |

### 9.3 默认数组

| 项 | 距离 | DriveMode | Slow Logic | Collision | Net |
|---|---:|---|---:|---|---:|
| LOD0 | 60 m | FlightController | 每帧 | QueryAndPhysics | 30 Hz |
| LOD1 | 150 m | PhysicsConstraint | 0.05 s | QueryAndPhysics | 15 Hz |
| LOD2 | 500 m | Kinematic | 0.10 s | QueryOnly | 8 Hz |
| LOD3 | 无限 | None | 关闭 | Disabled | 2 Hz + Dormancy |

### 9.4 配置原则

- 玩家控制、战斗、开火、近期受伤、任务关键、外部物理约束和 MustRemainPhysical 会选择数组第 0 项；
- 因此第 0 项应代表玩法最高优先级，而不只是“最近距离”；
- 玩家控制机型通常把第 0 项设为 FlightController；
- Root Motion 显式请求的 DriveMode 必须在数组中存在；
- 手动 DriveMode Override 必须能在数组中找到对应项；
- PhysicsConstraint 需要物理开启；
- Kinematic 和 None 会关闭物理；
- 最后一个条目必须能作为所有极远情况的兜底。

## 10. 调试参数

### 10.1 FlightController Component

| 参数 | 默认值 | 作用 |
|---|---:|---|
| `bEnableDebugLog` | false | 启用飞控周期诊断 |
| `bLogRotorCommands` | false | 输出旋翼命令 |
| `bLogRotorLayout` | false | 输出旋翼布局和权限 |
| `bLogSignDiagnostics` | false | 输出轴符号诊断 |
| `DebugLogIntervalSeconds` | 0.20 s | 日志间隔 |

日志只在需要时短期开启，尤其不要为大量 LOD0 NPC 全局开启。

### 10.2 Airscrew Component

| 参数 | 默认值 | 作用 |
|---|---:|---|
| `bDrawDebug` | false | 推力箭头 |
| `bDrawDebugText` | false | 命令、RPM、推力、反扭矩文字 |
| `DebugForceScale` | 0.1 | 推力箭头显示比例 |
| `DebugAxisLength` | 30 | 无推力时轴向显示长度 |
| `DebugTextOffset` | 18 | 文字偏移 |
| `DebugEnabledColor` | 绿色 | 活跃颜色 |
| `DebugDisabledColor` | 灰色 | 非活跃颜色 |

只有当前 LOD 的 `bAllowDebugDraw` 和 Airscrew 的 `bDrawDebug` 同时为 true 时组件才 Tick 绘制。

## 11. 推荐完整调参流程

### 阶段 A：物理与旋翼

1. 确认根 Mesh、Physics Asset、质量、质心、惯量和碰撞。
2. 确认模型 `ForwardAxis` 与 `+Z Up`。
3. 确认所有旋翼位置、推力轴、唯一名称和 CW/CCW。
4. 校准最大推力和悬停总距。
5. 校准偏航反扭矩。
6. 调电机 SpinUp/SpinDown/Slew。

通过标准：

- 对称机体悬停时电机指令大致对称；
- 不依靠极端 PID 就能产生足够升力和三轴力矩；
- 无轴向符号错误。

### 阶段 B：内环

1. 固定 FlightController 驱动。
2. 调 Roll/Pitch/Yaw Rate PID。
3. 调角阻尼前馈比例。
4. 调 Quaternion Attitude Gains。
5. 调姿态参考模型。

通过标准：

- 小幅姿态命令快速收敛；
- 无持续高频抖动；
- 旋翼不过度饱和；
- 松开姿态输入后能稳定回正。

### 阶段 C：高度与水平速度

1. 调 Vertical Velocity 环。
2. 调 Altitude 环。
3. 调水平 Velocity 环。
4. 调线性/垂直阻尼前馈。
5. 调 Position 环。
6. 验证 PositionHold 松杆制动和锁点。

### 阶段 D：Autopilot

1. 低速 MoveTo，使用 StopAndComplete。
2. 调 Accel、Decel、Jerk。
3. 调到达容差和稳定时间。
4. 调 FixedYaw 和 FaceTarget。
5. 调 PiecewiseLinear + PurePursuit。
6. 再测试 Orbit、CircleArc、Bezier、MinimumSnap。
7. 最后启用悬停推力估计和协调转弯。

### 阶段 E：替代驱动与 LOD

1. 单独固定 PhysicsConstraint 并调 Drive。
2. 单独固定 Kinematic 并调纠偏。
3. 测试三种驱动间速度和姿态连续性。
4. 配置距离、滞回和停留时间。
5. 测试 Gameplay 重要性提升。
6. 测试 Root Motion 临时驱动请求。
7. 最后测试服务器、Simulated Proxy、网络休眠和恢复。

## 12. 现象定位表

| 现象 | 优先检查 |
|---|---|
| 无人机完全不动 | Controller Profile、Controller Enabled、Armed、当前 DriveMode、旋翼 Profile、Autopilot 是否错误接管 |
| 玩家输入无效 | Pawn 是否本地占有、IMC/IA 是否赋值、是否仍启用 Autopilot、LOD0 是否 FlightController |
| 前进变侧移 | `ForwardAxis`、IA_Move X/Y 映射 |
| 起飞后持续上升/下降 | HoverCollective、质量、总推力、垂直速度 PID |
| 恒速达不到 | 线性阻尼、Damping Reserve、MaxTilt、MaxAcceleration、Velocity Kff |
| 松杆后返回松手点 | PositionHold 制动阈值和当前代码是否实际使用手动输入路径 |
| 松杆滑行太远 | 线性阻尼过小、Velocity Kp/Kd 过弱、水平加速度不足 |
| 到点冲过头 | Decel 太小、Jerk 太小、Velocity 环跟不上、电机减速慢、Cruise 太高 |
| 到点来回摆 | Position Kp 过大、Velocity 阻尼不足、到达容差过严 |
| 路径直角切角 | Cruise/LookAhead 太大、可用横向加速度不足、PiecewiseLinear 尖角 |
| 路径蛇形 | LookAhead 太小、CrossTrackGain 太大、Velocity 环过强或阻尼不足 |
| 转弯掉高 | Tilt Compensation、推力余量、MaxBank、MaxCollective |
| Roll/Pitch 高频抖动 | Rate Kp/Kd、D 截止频率、角阻尼补偿、惯量、物理步长 |
| 偏航慢 | ReactionTorqueCoefficient、Yaw 权限、Yaw Rate PID、角阻尼、Yaw 硬限制 |
| 偏航自旋 | CW/CCW 布局、反扭矩符号、ForwardAxis/Yaw 符号、Yaw 增益 |
| Constraint 弹簧振荡 | Position Strength 过高、Velocity Strength 过低、Force Limit |
| Kinematic 穿透 | Sweep 未开或碰撞配置错误 |
| Root Motion 动画播放但不移动 | Montage Root Motion、AnimBP Slot、根 Skeletal Mesh、Autopilot Active、DriveMode |
| Root Motion 结束后长时间不完成 | Motion Constraints 太低、到达容差太严、最终目标受碰撞阻挡 |
| Root Motion 提交立即失败 | LOD 数组缺少请求 DriveMode、手动覆盖冲突、当前 DriveMode None、Montage 无 Root Motion |
| 远处 NPC 不更新 | LOD `bRunSlowLogic`、DriveMode None、网络休眠 |
| LOD 边界频繁切换 | 增加 Hysteresis 或 Minimum Residence |
| 故障后无法 Arm | Failure Policy 锁存仍在，先修复故障再 Reset Latch |

## 13. 当前不参与运行调节的结构

以下类型存在于代码中，但当前不属于实际飞行配置链，调整它们不会改变当前飞行结果：

| 类型 | 当前状态 |
|---|---|
| `FAircraftAerodynamicsConfig` | 未被 FlightController Profile 持有，风、分轴阻力和地效未接入 |
| `FAircraftSensorSuiteConfig` 及各传感器噪声配置 | 飞控直接读取 Chaos 真值 |
| `FAircraftEstimatorConfig` | 未接入运行时状态估计 |
| `FAircraftFailsafeConfig` | 未接入；旋翼权限故障使用 FlightController Failure Policy |
| `FAircraftFirstOrderFilterState` | 提供算法结构，但没有传感器运行链消费 |

不要把这些参数暴露给普通策划作为当前可用调节点。

## 14. 最终验收清单

### 机体

- 悬停、升降、前后左右、三轴旋转方向正确；
- 质量和推重比有足够余量；
- 旋翼命令无长期异常饱和；
- 不依赖极端物理阻尼维持稳定。

### 手动

- PositionHold 松杆先制动，再保持实际停止位置；
- 高度和航向松杆后稳定；
- 切换模式无明显积分冲击；
- Autopilot 关闭后输入立即恢复。

### 自动驾驶

- MoveTo 提前制动并满足到达判据；
- PassThrough 保持合理退出速度；
- FollowPath 各模式语义符合关卡需求；
- FaceTarget 可与移动目标独立；
- Orbit 正反方向、半径和速度正确；
- Timeout、Cancel、Replace 和目标 Actor 销毁均返回正确结果。

### Root Motion

- 普通 Montage 不创建 Intent；
- 三种 Root Motion 后端均可独立运行；
- 播放速率变化后仍可跟踪；
- 中断、超时、碰撞阻挡和 Montage 自然结束均有明确结果。

### LOD 与网络

- 玩家和战斗对象进入数组第 0 项；
- 距离切换无频繁抖动；
- DriveMode 切换位置、速度和姿态可接受；
- Simulated Proxy 不重复运行权威飞控；
- Dormancy 能正常进入并在重要性恢复时唤醒。

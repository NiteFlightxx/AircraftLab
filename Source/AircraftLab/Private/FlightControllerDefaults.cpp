#include "FlightControllerProfileAsset.h"

void FlightControllerConfig::InitializeDefaults(FAircraftFlightControllerConfig& OutConfig)
{
	// ========================================================================
	// 运动限制（安全边界）
	// 100kg 无人机倾斜过大会失控，需严格限制
	// ========================================================================
	OutConfig.Limits.MaxTiltAngleDegrees = 25.0f;         // 最大倾角（原35°，大惯性降至25°）
	OutConfig.Limits.MaxYawRateDegreesPerSec = 90.0f;     // 最大偏航角速率（°/s）
	OutConfig.Limits.MaxRollRateDegreesPerSec = 180.0f;   // 最大滚转角速率
	OutConfig.Limits.MaxPitchRateDegreesPerSec = 180.0f;  // 最大俯仰角速率
	OutConfig.Limits.MaxClimbRateCmPerSec = 300.0f;       // 最大爬升率（3 m/s）
	OutConfig.Limits.MaxDescentRateCmPerSec = 200.0f;     // 最大下降率（2 m/s）
	OutConfig.Limits.MaxHorizontalSpeedCmPerSec = 800.0f;  // 最大水平速度（8 m/s）
	OutConfig.Limits.MaxHorizontalAccelerationCmPerSecSq = 600.0f; // 最大水平加速度
	OutConfig.Limits.MaxVerticalAccelerationCmPerSecSq = 500.0f;   // 最大垂直加速度
	OutConfig.Limits.MinCollectiveCommand = 0.0f;        // 最小总距（0 = 零推力）
	OutConfig.Limits.HoverCollectiveCommand = 0.50f;      // 悬停总距（悬停点）
	OutConfig.Limits.MaxCollectiveCommand = 1.0f;        // 最大总距（满推力）

	// ========================================================================
	// Position PID — 外环：位置误差 → 期望速度
	// 公式：v_des = Kp·(pos_held − pos_current) + Kff·v_setpoint
	// 注意这是 P 控制器（Ki=0, Kd 提供速度阻尼）
	// Kd 项 = Kd·d(error)/dt ≈ Kd·(−v_current)，等效于速度阻尼
	// Kff=1.0 激活速度前馈通道：Autopilot 注入时直接用设定速度驱动，消除跟踪滞后
	//   手动模式 Kff 无副作用（FeedForwardInput=0，Kff·0=0）
	// 输出限制 = MaxSpeed，确保期望速度不超物理极限
	// ========================================================================
	OutConfig.Position.PositionGains.X = { 0.40f, 0.0f, 0.30f, 0.0f, OutConfig.Limits.MaxHorizontalSpeedCmPerSec };
	OutConfig.Position.PositionGains.Y = { 0.40f, 0.0f, 0.30f, 0.0f, OutConfig.Limits.MaxHorizontalSpeedCmPerSec };
	OutConfig.Position.PositionGains.X.Kff = 1.0f; // 速度前馈（Autopilot 位置环）
	OutConfig.Position.PositionGains.Y.Kff = 1.0f;
	// ========================================================================
	// Velocity PID — 内环：速度误差 → 期望加速度
	// 公式：a_des = Kp·(v_des − v_current) + Ki·∫(v_des − v)dt + Kd·d(v_des − v)/dt + Kff·a_setpoint
	// Kff=1.0 激活加速度前馈通道：Autopilot 注入时直接用设定加速度驱动
	// 输出限制 = MaxHorizontalAcceleration（仅 X/Y）。Z 轴由 Altitude 配置独立控制。
	// ========================================================================
	OutConfig.Position.VelocityGains.X = { 1.50f, 0.01f, 0.60f, 3000.0f, OutConfig.Limits.MaxHorizontalAccelerationCmPerSecSq };
	OutConfig.Position.VelocityGains.Y = { 1.50f, 0.01f, 0.60f, 3000.0f, OutConfig.Limits.MaxHorizontalAccelerationCmPerSecSq };
	OutConfig.Position.VelocityGains.X.Kff = 1.0f; // 加速度前馈（Autopilot 速度环）
	OutConfig.Position.VelocityGains.Y.Kff = 1.0f;
	OutConfig.Position.LinearDampingFeedForwardScale = 1.0f;
	OutConfig.Position.DampingAccelerationReserveFraction = 0.2f;
	// 微分截止频率降低 → 更强滤波 → 减少角速率噪声引起的抖动
	OutConfig.Position.VelocityGains.X.DerivativeCutoffHz = 12.0f;
	OutConfig.Position.VelocityGains.Y.DerivativeCutoffHz = 12.0f;

	// ========================================================================
	// Angle PID — 外环：倾角误差 → 期望角速率
	// 公式：ω_des = Kp·(θ_des − θ_current) + Kd·d(θ_error)/dt
	// Yaw 轴 Kff=1.0 激活偏航角速度前馈通道（Autopilot 协调转弯/路径跟踪航向）
	// 输出限制 = MaxRate（与速率内环的输入范围匹配）
	// Roll/Pitch/Yaw 均由四元数误差比例控制。
	// ========================================================================
	OutConfig.Attitude.QuaternionAttitudeGains.Roll = 4.5f;
	OutConfig.Attitude.QuaternionAttitudeGains.Pitch = 4.5f;
	OutConfig.Attitude.QuaternionAttitudeGains.Yaw = 3.0f;

	// ========================================================================
	// Rate PID — 内环：角速率误差 → 归一化力矩指令
	// 公式：u = PID(ω_des − ω_current) + normalized angular-damping feed-forward
	// 速率环用 UpdateFromMeasurement（导数对测量值），避免设定值阶跃时的 kick
	// 输出限制 = 0.35（归一化，对应混合器中该轴最大权限的 35%）
	// 第 6 批调参（Bug #5 修复后）：速率环增益提升 4×。
	//   Bug #5 修复前，四元数姿态环期望角速率被砍 57×（量纲错误），速率环在
	//   ~0.06°/s 量级的期望值下勉强够用。修复后姿态环输出正确量级（瞬态可达
	//   22°/s），但旧 Kp=0.002 在 22°/s 误差下仅产出 0.044 轴指令——速率环
	//   无法跟踪姿态环设定值，积分器需 ~30s 建立物理配平，导致缓慢漂移。
	//   提升 4× 后：满 P 权限对应 44°/s 误差（原 175°/s），积分器 ~3s 建立配平。
	//   OutputLimit=0.35 仍是安全网，不会因增益增大而过驱。
	// 参考模型前馈已合入角速度设定值；角速度环不再暴露重复 Kff。
	// ========================================================================
	OutConfig.Attitude.RateGains.Roll = { 0.0080f, 0.00100f, 0.00040f, 120.0f, 0.35f };
	OutConfig.Attitude.RateGains.Pitch = { 0.0080f, 0.00100f, 0.00040f, 120.0f, 0.35f };
	OutConfig.Attitude.RateGains.Yaw = { 0.0012f, 0.00015f, 0.00008f, 120.0f, 0.20f };
	// 角速度环不暴露 Kff：参考模型前馈已包含在角速度设定值中，运行时角阻尼补偿
	// 使用独立的归一化力矩前馈，并通过 PID 内部同一限幅/anti-windup 路径合成。
	// 参考模型导数 rate_ff（°/s，可达 ±100）已在角度环以 Kff=1.0 注入 DesiredRate（四元数路径
	// 直接 +RollRateFF）。角速度环以 DesiredRate 为设定值，通过 Kp·(DesiredRate−ω) 跟踪即可——
	// FF 已含在设定值中。若角速度环再开 Kff，则 rate_ff 被二次叠加：
	//   1) Kp_rate·rate_ff（经设定值）+ 2) Kff_rate·rate_ff（FF 通道）
	// 且 rate_ff 量纲为 °/s（最大 100），Kff_rate=0.5 会产出 50 的归一化输出，
	// 远超 OutputLimit=0.35 → 角速度环被 FF 永久饱和 → 过冲 → 极限环振荡。
	// 对标 PX4：rate setpoint 已含 FF，rate controller 自身 Kff=0，仅 Kp 跟踪。
	OutConfig.Attitude.AngularDampingFeedForwardScale = 1.0f;
	OutConfig.Attitude.RateGains.Roll.DerivativeCutoffHz = 18.0f;
	OutConfig.Attitude.RateGains.Pitch.DerivativeCutoffHz = 18.0f;
	OutConfig.Attitude.RateGains.Yaw.DerivativeCutoffHz = 15.0f;

	// ========================================================================
	// Altitude PID — 高度控制（与垂直通道并行）
	// 外环：高度误差 → 期望垂直速度
	//   v_z_des = Kp·(z_held − z_current) + Kd·d(z_error)/dt + Kff·v_z_setpoint
	//   Kff=1.0 激活垂直速度前馈通道（Autopilot 高度环）
	// 内环：垂直速度误差 → 总距偏移（不暴露 Kff，推力前馈走基准偏移）
	//   Δc = Kp·(v_z_des − v_z_current) + Ki·∫(v_z_des − v_z)dt + Kd·d(v_z_des − v_z)/dt
	//   Collective = Clamp(HoverCollective + Δc, Min, Max)         （手动）
	//   Collective = Clamp(ThrustFeedForward + Δc, Min, Max)        （Autopilot）
	// ========================================================================
	OutConfig.Altitude.AltitudeGains = { 1.20f, 0.0f, 0.20f, 0.0f, OutConfig.Limits.MaxClimbRateCmPerSec };
	OutConfig.Altitude.AltitudeGains.Kff = 1.0f; // 垂直速度前馈（Autopilot 高度环）
	OutConfig.Altitude.VerticalVelocityGains = { 0.0015f, 0.00020f, 0.00050f, 2500.0f, 0.30f };
	OutConfig.Altitude.VerticalVelocityGains.DerivativeCutoffHz = 10.0f;
	OutConfig.Altitude.VerticalDampingFeedForwardScale = 1.0f;

	// ========================================================================
	// 控制分配参数
	// λ = DampedPseudoInverseLambda — 阻尼系数
	// 公式中的 λ² 项加在法矩阵对角线上，防止 J·J^T 接近奇异时解爆炸
	// 默认 0.05：轻微正则化，几乎不影响正常工况，但在权限极低时防止数值爆炸
	// 倾斜补偿开关与 MinCosTilt 取结构体默认值。
	// ========================================================================
	OutConfig.Allocator.DampedPseudoInverseLambda = 0.05f;
}


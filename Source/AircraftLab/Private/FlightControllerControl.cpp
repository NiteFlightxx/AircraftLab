#include "FlightControllerRuntimeObjects.h"

#include "Math/RotationMatrix.h"

float FFlightControlSolver::ComputeVerticalControl(FFlightControlSolverContext& Context, const FDronePilotInput& PilotInput, float DeltaSeconds, float& OutDesiredVerticalVelocity)
{
	const float MinCollective = Context.Config.Controller.Limits.MinCollectiveCommand;
	const float HoverCollective = Context.Config.Controller.Limits.HoverCollectiveCommand;
	const float MaxCollective = Context.Config.Controller.Limits.MaxCollectiveCommand;
	const float CurrentAltitude = Context.Runtime.EstimatedState.State.PositionCm.Z;
	const float CurrentVerticalVelocity = Context.Runtime.EstimatedState.State.VelocityCmPerSec.Z;

	if (!Context.ModeCapabilities.CanHoldAltitude)
	{
		// ---- 路径 A：无高度保持 ----
		// 重置 PID 状态，避免残留积分项
		Context.Runtime.HoldTargets.bAltitudeHoldInitialized = false;
		PidStates.Altitude.Reset();
		PidStates.VerticalVelocity.Reset();
		// 油门杆 → 垂直速度（线性映射）
		OutDesiredVerticalVelocity = FMath::GetMappedRangeValueClamped(
			FVector2D(-1.0f, 1.0f),
			FVector2D(-Context.Config.Controller.Limits.MaxDescentRateCmPerSec, Context.Config.Controller.Limits.MaxClimbRateCmPerSec),
			PilotInput.Throttle);
		// 油门杆 → 总距（悬停点为中心的线性映射）
		return MapCenteredThrottleToCollective(Context, PilotInput.Throttle);
	}

	// 高度保持初始化（手动路径用；Autopilot 路径直接使用设定值，忽略此锁定值）
	if (!Context.Runtime.HoldTargets.bAltitudeHoldInitialized)
	{
		Context.Runtime.HoldTargets.HeldAltitudeCm = CurrentAltitude;
		Context.Runtime.HoldTargets.bAltitudeHoldInitialized = true;
		PidStates.Altitude.Reset();
		PidStates.VerticalVelocity.Reset();
	}

	// ---- 路径 C：Autopilot 注入 ----
	if (Context.bUseAutopilotSetpoint && Context.AutopilotInjection.bValid)
	{
		const FAutopilotInjection& AI = Context.AutopilotInjection;
		// 高度外环：设定值=AltitudeSetpointCm，前馈=垂直速度设定值（Kff 通道）
		OutDesiredVerticalVelocity = PidStates.Altitude.UpdateFromMeasurement(
			AI.AltitudeSetpointCm, CurrentAltitude, DeltaSeconds,
			Context.Config.Controller.Altitude.AltitudeGains, AI.VerticalVelocitySetpointCmPerSec);
		OutDesiredVerticalVelocity = FMath::Clamp(OutDesiredVerticalVelocity,
			-Context.Config.Controller.Limits.MaxDescentRateCmPerSec, Context.Config.Controller.Limits.MaxClimbRateCmPerSec);
		// 垂直速度内环（无前馈，推力前馈走基准偏移而非 Kff）
		const float CollectiveOffset = PidStates.VerticalVelocity.UpdateFromMeasurement(
			OutDesiredVerticalVelocity, CurrentVerticalVelocity, DeltaSeconds,
			Context.Config.Controller.Altitude.VerticalVelocityGains);
		// 推力前馈作总距基准（含重力补偿），替代 HoverCollective
		return FMath::Clamp(AI.ThrustFeedForward + CollectiveOffset, MinCollective, MaxCollective);
	}

	// ---- 路径 B（手动）：高度保持 ----
	// RTH/AutoLand 内联已删除，由 BehaviorPlanner 经 TrajectoryGenerator 驱动
	{
		// 油门杆在死区外 → 手动爬升/下降率，重新锚定高度
		const float ThrottleMagnitude = FMath::Abs(PilotInput.Throttle);
		if (ThrottleMagnitude > Context.Config.Input.VerticalHoldStickDeadband)
		{
			// 将死区外的输入线性映射到 [0,1]
			const float NormalizedInput = (ThrottleMagnitude - Context.Config.Input.VerticalHoldStickDeadband)
				/ FMath::Max(1.0f - Context.Config.Input.VerticalHoldStickDeadband, UE_SMALL_NUMBER);
			const float SignedInput = NormalizedInput * FMath::Sign(PilotInput.Throttle);
			// 根据方向选择最大速率
			const float MaxVerticalRate = SignedInput >= 0.0f ? Context.Config.Controller.Limits.MaxClimbRateCmPerSec : Context.Config.Controller.Limits.MaxDescentRateCmPerSec;
			OutDesiredVerticalVelocity = SignedInput * MaxVerticalRate;
			// 重新锚定高度到当前位置（松手后将保持新高度）
			Context.Runtime.HoldTargets.HeldAltitudeCm = CurrentAltitude;
			PidStates.Altitude.Reset();
		}
		else
		{
			// 油门杆在死区内 → 高度 PID 保持锁定高度
			// PID_alt: v_z_des = Kp·(z_held − z) + Kd·d(z_error)/dt
			// 使用 UpdateFromMeasurement（导数对测量值），避免高度设定值跳变时的 kick
			OutDesiredVerticalVelocity = PidStates.Altitude.UpdateFromMeasurement(
				Context.Runtime.HoldTargets.HeldAltitudeCm, CurrentAltitude, DeltaSeconds, Context.Config.Controller.Altitude.AltitudeGains);
			OutDesiredVerticalVelocity = FMath::Clamp(OutDesiredVerticalVelocity,
				-Context.Config.Controller.Limits.MaxDescentRateCmPerSec, Context.Config.Controller.Limits.MaxClimbRateCmPerSec);
		}
	}

	// ---- 垂直速度内环（手动路径）----
	// PID_vz: Δc = Kp·(v_z_des − v_z) + Ki·∫(v_z_des − v_z)dt + Kd·d(v_z_des − v_z)/dt
	// 输出 Δc 是总距偏移量，加在悬停点上
	const float CollectiveOffset = PidStates.VerticalVelocity.UpdateFromMeasurement(
		OutDesiredVerticalVelocity, CurrentVerticalVelocity, DeltaSeconds, Context.Config.Controller.Altitude.VerticalVelocityGains);
	// 最终总距 = 悬停总距 + PID偏移，限制在 [Min, Max]
	return FMath::Clamp(HoverCollective + CollectiveOffset, MinCollective, MaxCollective);
}


FRotator FFlightControlSolver::ComputeDesiredAttitude(FFlightControlSolverContext& Context, const FDronePilotInput& PilotInput, float DeltaSeconds)
{
	if (!Context.ModeCapabilities.CanUseVelocityControl)
	{
		// ---- 路径 A：摇杆直接映射 ----
		Context.Runtime.HoldTargets.bPositionHoldInitialized = false;
		PidStates.Position.X.Reset(); PidStates.Position.Y.Reset();
		PidStates.Velocity.X.Reset(); PidStates.Velocity.Y.Reset();
		const float ManualRollDegrees = PilotInput.Roll * Context.Config.Controller.Limits.MaxTiltAngleDegrees;
		const float ManualPitchDegrees = -PilotInput.Pitch * Context.Config.Controller.Limits.MaxTiltAngleDegrees;
		return FRotator(ManualPitchDegrees, Context.Runtime.EstimatedState.State.AttitudeDegrees.Yaw, ManualRollDegrees);
	}

	// ---- 路径 B：速度/位置 PID → 悬停倾斜方程 ----
	const FVector DesiredHorizontalAcceleration = ComputeDesiredHorizontalAcceleration(Context, PilotInput, DeltaSeconds);
	const float GravityMagnitude = Context.PhysicsCache.GravityMagnitudeCmPerSecSq;

	// 构造仅含航向的"平面旋转"——提取机体前/右方向的水平投影
	const FRotator FlatYawRotation(0.0f, Context.Runtime.EstimatedState.State.AttitudeDegrees.Yaw, 0.0f);
	const FVector ForwardFlat = FRotationMatrix(FlatYawRotation).GetUnitAxis(EAxis::X);
	const FVector RightFlat = FRotationMatrix(FlatYawRotation).GetUnitAxis(EAxis::Y);

	// 将期望加速度投影到机体前/右方向
	const float ForwardAcceleration = FVector::DotProduct(DesiredHorizontalAcceleration, ForwardFlat);
	const float RightAcceleration = FVector::DotProduct(DesiredHorizontalAcceleration, RightFlat);

	// 悬停倾斜方程：tan(θ) = a/g
	//   θ_pitch = −atan2(a_forward, g)  （取负：前加速=低头=负俯仰）
	//   φ_roll  =  atan2(a_right,  g)
	float DesiredPitchDegrees = -FMath::RadiansToDegrees(FMath::Atan2(ForwardAcceleration, GravityMagnitude));
	float DesiredRollDegrees = FMath::RadiansToDegrees(FMath::Atan2(RightAcceleration, GravityMagnitude));

	// Autopilot 协调转弯滚转叠加（TurnBehavior 输出，叠加在悬停倾斜方程之上）
	if (Context.bUseAutopilotSetpoint && Context.AutopilotInjection.bValid)
	{
		DesiredRollDegrees += Context.AutopilotInjection.TurnRollDegrees;
	}

	// 限制最大倾角——超出此角度可能推力不足以抵消重力分量
	DesiredRollDegrees = FMath::Clamp(DesiredRollDegrees, -Context.Config.Controller.Limits.MaxTiltAngleDegrees, Context.Config.Controller.Limits.MaxTiltAngleDegrees);
	DesiredPitchDegrees = FMath::Clamp(DesiredPitchDegrees, -Context.Config.Controller.Limits.MaxTiltAngleDegrees, Context.Config.Controller.Limits.MaxTiltAngleDegrees);
	return FRotator(DesiredPitchDegrees, Context.Runtime.EstimatedState.State.AttitudeDegrees.Yaw, DesiredRollDegrees);
}


float FFlightControlSolver::ComputeDesiredYawRate(FFlightControlSolverContext& Context, const FDronePilotInput& PilotInput, float DeltaSeconds)
{
	// ---- 路径 C：Autopilot 注入 ----
	if (Context.bUseAutopilotSetpoint && Context.AutopilotInjection.bValid)
	{
		const FAutopilotInjection& AI = Context.AutopilotInjection;
		// 偏航角 PID：设定值=YawSetpointDegrees，前馈=偏航角速度设定值（Kff 通道）
		const float YawError = FRotator::NormalizeAxis(
			AI.YawSetpointDegrees - Context.Runtime.EstimatedState.State.AttitudeDegrees.Yaw);
		const float DesiredYawRate = PidStates.Angle.Yaw.UpdateFromError(
			YawError, DeltaSeconds, Context.Config.Controller.Attitude.AngleGains.Yaw, AI.YawRateSetpointDegPerSec);
		return FMath::Clamp(DesiredYawRate,
			-Context.Config.Controller.Limits.MaxYawRateDegreesPerSec, Context.Config.Controller.Limits.MaxYawRateDegreesPerSec);
	}

	// ---- 手动路径 ----
	// 手动偏航角速率
	const float ManualYawRate = PilotInput.Yaw * Context.Config.Controller.Limits.MaxYawRateDegreesPerSec;

	if (!Context.ModeCapabilities.CanHoldYaw)
	{
		// 无偏航保持：直接输出手动速率
		Context.Runtime.HoldTargets.bYawHoldInitialized = false;
		PidStates.Angle.Yaw.Reset();
		return ManualYawRate;
	}

	// 摇杆超出死区 → 手动偏航率，同时重新锁定航向
	if (FMath::Abs(PilotInput.Yaw) > Context.Config.Input.YawHoldStickDeadband)
	{
		Context.Runtime.HoldTargets.HeldYawDegrees = Context.Runtime.EstimatedState.State.AttitudeDegrees.Yaw;
		Context.Runtime.HoldTargets.bYawHoldInitialized = true;
		PidStates.Angle.Yaw.Reset();
		return ManualYawRate;
	}

	// 初始化锁定航向
	if (!Context.Runtime.HoldTargets.bYawHoldInitialized)
	{
		Context.Runtime.HoldTargets.HeldYawDegrees = Context.Runtime.EstimatedState.State.AttitudeDegrees.Yaw;
		Context.Runtime.HoldTargets.bYawHoldInitialized = true;
		PidStates.Angle.Yaw.Reset();
	}

	// 偏航角 PID 锁定航向
	// ψ_err = NormalizeAxis(ψ_held − ψ_current)  映射到 [−180, 180]
	// ψ̇_des = PID_yaw(ψ_err) — 使用 UpdateFromError，因为角度环设定值是阶跃的（手动改目标时已 Reset）
	const float YawError = FRotator::NormalizeAxis(Context.Runtime.HoldTargets.HeldYawDegrees - Context.Runtime.EstimatedState.State.AttitudeDegrees.Yaw);
	const float DesiredYawRate = PidStates.Angle.Yaw.UpdateFromError(YawError, DeltaSeconds, Context.Config.Controller.Attitude.AngleGains.Yaw);
	return FMath::Clamp(DesiredYawRate, -Context.Config.Controller.Limits.MaxYawRateDegreesPerSec, Context.Config.Controller.Limits.MaxYawRateDegreesPerSec);
}


FVector FFlightControlSolver::ComputeDesiredBodyRates(FFlightControlSolverContext& Context, const FDronePilotInput& PilotInput, const FRotator& DesiredAttitude, float DesiredYawRate, float DeltaSeconds)
{
	const FRotator CurrentAttitude = Context.Runtime.EstimatedState.State.AttitudeDegrees;
	// 计算滚转/俯仰误差，NormalizeAxis 确保在 [−180, 180] 范围内
	const float RollError = FRotator::NormalizeAxis(DesiredAttitude.Roll - CurrentAttitude.Roll);
	const float PitchError = FRotator::NormalizeAxis(DesiredAttitude.Pitch - CurrentAttitude.Pitch);

	// Acro/Manual 模式的默认值：摇杆直通
	float DesiredRollRate = PilotInput.Roll * Context.Config.Controller.Limits.MaxRollRateDegreesPerSec;
	float DesiredPitchRate = -PilotInput.Pitch * Context.Config.Controller.Limits.MaxPitchRateDegreesPerSec;
	// 角速度前馈（第 3 批：由参考模型导数产生，供角速度环 Kff 通道消费）
	float RollRateFF = 0.0f;
	float PitchRateFF = 0.0f;

	// Angle 模式：角度环覆盖默认值
	if (Context.Runtime.AttitudeMode != EDroneAttitudeMode::Acro && Context.Runtime.AttitudeMode != EDroneAttitudeMode::Manual)
	{
		// ---- 第 3 批：2 阶临界阻尼参考模型（对标 PX4 AttitudeControl.cpp:82-129）----
		// 对期望 Roll/Pitch 设定值做平滑：ẍ + 2ω·ẋ + ω²·(x − x_sp) = 0，ζ=1 临界阻尼。
		// 输出平滑设定值 x_smooth 及其导数 v=ẋ（角速度前馈 rate_ff）。
		// 角速度设定值 = Kp·(x_smooth − current) + rate_ff，前馈承担"已知运动学"部分，
		// PID 只补模型误差，Kp 可降低、过冲减小。
		const FDroneAttitudeControllerConfig& AttCfg = Context.Config.Controller.Attitude;
		float SmoothedRoll = DesiredAttitude.Roll;
		float SmoothedPitch = DesiredAttitude.Pitch;

		if (AttCfg.bEnableAttitudeRefModel)
		{
			const float Omega = FMath::Max(AttCfg.RefModelNaturalFrequency, UE_SMALL_NUMBER);
			const float FFLimit = AttCfg.RefModelRateFFLimitDegPerSec;
			// ZOH 离散积分（半隐式 Euler，稳定且简单）：
			//   v += ω²·(x_sp − x)·dt − 2ω·v·dt
			//   x += v·dt
			auto StepRefModel = [Omega, DeltaSeconds](FFlightControlReferenceModelState& S, float Setpoint)
			{
				if (!S.bInitialized) { S.x = Setpoint; S.v = 0.0f; S.bInitialized = true; return; }
				const float Accel = Omega * Omega * (Setpoint - S.x) - 2.0f * Omega * S.v;
				S.v += Accel * DeltaSeconds;
				S.x += S.v * DeltaSeconds;
			};
			StepRefModel(RollReferenceModel, DesiredAttitude.Roll);
			StepRefModel(PitchReferenceModel, DesiredAttitude.Pitch);
			SmoothedRoll = RollReferenceModel.x;
			SmoothedPitch = PitchReferenceModel.x;
			RollRateFF = FMath::Clamp(RollReferenceModel.v, -FFLimit, FFLimit);
			PitchRateFF = FMath::Clamp(PitchReferenceModel.v, -FFLimit, FFLimit);
		}

		// 角度环 PID：误差基于【平滑后】设定值，前馈 = 参考模型导数（注入 Kff 通道）
		// p_des = Kp·(x_smooth − current) + Kd·d(err)/dt + Kff·rate_ff
		const float SmoothedRollError = FRotator::NormalizeAxis(SmoothedRoll - CurrentAttitude.Roll);
		const float SmoothedPitchError = FRotator::NormalizeAxis(SmoothedPitch - CurrentAttitude.Pitch);

		if (AttCfg.bEnableQuaternionAttitude)
		{
			// ---- 第 5 批：四元数姿态误差 + 推力方向优先（对标 PX4 AttitudeControl.cpp:139-205）----
			// Q_des = 由平滑后 Roll/Pitch + 当前 Yaw 构造（Yaw 由 DesiredYawRate 单独处理）
			// Q_err = Q_cur⁻¹ · Q_des → 提取机体角速度设定值（消除欧拉角耦合）
			// 推力方向优先：Roll/Pitch 误差全权，Yaw 误差按 YawWeight 缩放
			const FQuat QCur = CurrentAttitude.Quaternion();
			const FQuat QDes = FRotator(SmoothedPitch, CurrentAttitude.Yaw, SmoothedRoll).Quaternion();
			FQuat QErr = QCur.Inverse() * QDes;
			// 取最短路径（w<0 时取反，避免大角度冗余旋转）
			if (QErr.W < 0.0f) QErr = FQuat(-QErr.X, -QErr.Y, -QErr.Z, -QErr.W);
			QErr.Normalize();

			// 小角度近似：ω_sp = 2 · q_err.imag · Kp（q_err 在机体系）
			// 符号约定对齐（修复日志 Bug #3：俯仰符号翻转致前漂发散）：
			//   q_err.imag 来自 QCur⁻¹·QDes，处于与 Chaos 相同的右手机体系——
			//   绕 X 正向=左滚、绕 Y 正向=低头、绕 Z 正向=右偏。
			//   但角速度【测量】在 UpdateEstimatedState_PhysicsThread 已对 X/Y 取负
			//   （FVector(-X,-Y,Z)），转为飞控的 d(angle)/dt 约定（正向=右滚/抬头/右偏）。
			//   因此期望角速率须同样对 X/Y 取负、Z 不取负，才能与测量同号、角速度环
			//   形成负反馈。修复前用 +2·QErr.X/Y 致 Roll/Pitch 期望角速率符号翻转：
			//   俯仰案例——期望俯仰 +25°(抬头制动前漂)，角度误差 +56°，QErr.Y 为负，
			//   旧代码输出 -4.25°/s(低头)，无人机反而低头、前漂加速；符号修复后输出 +4.22°/s
			//   （与日志 4.25 吻合）。注意：此仅验证【符号】正确——4.22°/s 本身比欧拉路径
			//   4.5×56°=252°/s 小约 57 倍，是【量纲】缺陷（见下方 RadiansToDegrees 修复 Bug #5）。
			//   偏航测量未取负 Z，故 QErr.Z 保持 +2 不变。
			const float YawW = FMath::Clamp(AttCfg.YawWeight, 0.0f, 1.0f);
			const float KpRoll  = Context.Config.Controller.Attitude.AngleGains.Roll.Kp;
			const float KpPitch = Context.Config.Controller.Attitude.AngleGains.Pitch.Kp;
			const float KpYaw   = Context.Config.Controller.Attitude.AngleGains.Yaw.Kp;
			// Roll/Pitch：全权对齐推力方向 + 参考模型前馈。
			// X/Y 取负（与角速度测量约定对齐，见上方块注释），Z 不取负。
			//
			// 量纲修正（Bug #5：四元数期望角速率量纲不符，纠偏偏弱 ~57× 致缓慢发散）：
			//   2·q_err.imag 为无量纲量（小角度下 ≈ 误差弧度），× Kp(1/s) 得 rad/s。
			//   但下游（角速度环、测量、限幅、RollRateFF/PitchRateFF）全部以 deg/s 为单位，
			//   且 KpRoll/KpPitch=4.5 是按【欧拉路径】度数误差标定的（4.5×34°=153°/s）。
			//   若直接把 2·QErr·Kp 当 deg/s，34° 误差仅得 2·sin(17°)·4.5≈2.63°/s，
			//   比欧拉路径小 180/π≈57.3 倍，角速度环被严重"饿死"——表现为起飞旋翼起转
			//   瞬态扰动后纠偏过慢、单调发散（俯仰持续低头、前漂累积、期望角速率偏小）。
			//   修复：RadiansToDegrees 把四元数项转 deg/s，与前馈及下游量纲对齐。
			//   符号（Bug #3 取负）不变——RadiansToDegrees 是正比例，不改变符号。
			DesiredRollRate  = FMath::RadiansToDegrees(-2.0f * QErr.X * KpRoll)  + RollRateFF;
			DesiredPitchRate = FMath::RadiansToDegrees(-2.0f * QErr.Y * KpPitch) + PitchRateFF;
			// Yaw：四元数误差提供纠偏项，按 YawWeight 缩放叠加到外部给定偏航率。
			// 偏航测量未取负 Z（见 UpdateEstimatedState_PhysicsThread），故 QErr.Z 保持 +2。
			// （推力方向优先：YawWeight 小→偏航纠偏弱→优先保 Roll/Pitch）
			// 量纲同 Roll/Pitch：2·QErr·Kp 为 rad/s，需 RadiansToDegrees 转 deg/s。
			DesiredYawRate += FMath::RadiansToDegrees(2.0f * QErr.Z * KpYaw * YawW);
			// 注：四元数路径直接产出角速度设定值，不经角度 PID（避免冗余积分累积）。
			//   角度 PID 状态在此路径下保持冻结（ResetControllerState 时清零），仅欧拉路径推进。
		}
		else
		{
			// 欧拉角线性误差路径（第 3 批原始路径，向后兼容）
			DesiredRollRate = PidStates.Angle.Roll.UpdateFromError(SmoothedRollError, DeltaSeconds, Context.Config.Controller.Attitude.AngleGains.Roll, RollRateFF);
			DesiredPitchRate = PidStates.Angle.Pitch.UpdateFromError(SmoothedPitchError, DeltaSeconds, Context.Config.Controller.Attitude.AngleGains.Pitch, PitchRateFF);
		}
	}

	// 缓存角速度前馈供角速度环 Kff 通道消费（第 3 批）
	RateFeedForwardDegPerSec = FVector(RollRateFF, PitchRateFF, 0.0f);

	// 限幅到最大角速率
	DesiredRollRate = FMath::Clamp(DesiredRollRate, -Context.Config.Controller.Limits.MaxRollRateDegreesPerSec, Context.Config.Controller.Limits.MaxRollRateDegreesPerSec);
	DesiredPitchRate = FMath::Clamp(DesiredPitchRate, -Context.Config.Controller.Limits.MaxPitchRateDegreesPerSec, Context.Config.Controller.Limits.MaxPitchRateDegreesPerSec);
	return FVector(DesiredRollRate, DesiredPitchRate, DesiredYawRate);
}


FVector FFlightControlSolver::ComputeBodyTorqueCommand(FFlightControlSolverContext& Context, const FVector& DesiredBodyRatesDegreesPerSec, float DeltaSeconds)
{
	const FVector CurrentBodyRates = Context.Runtime.EstimatedState.State.AngularVelocityBodyDegreesPerSec;

	// 第 3 批：角速度前馈注入 Kff 通道。
	// RateFeedForwardDegPerSec 由 ComputeDesiredBodyRates 的参考模型导数填入（Roll/Pitch），
	// Yaw 通道前馈置零（偏航前馈已由 AngleGains.Yaw.Kff 在角度环承载）。

	// ---- 第 4 批：分配饱和回传抗 windup（对标 PX4 rate_control.cpp:88-117）----
	// 上一帧 AllocateToRotors 算出的饱和标志（1 帧延迟，可接受）。
	// 当某轴正/负方向分配饱和（残差>0/<0）时，禁止该方向角速度误差继续累积积分，
	// 避免积分项在"物理上无法满足"的方向上无限增长。
	// 实现：复制该轴增益并把 Ki 置零（仅在饱和方向），其余项（Kp/Kd/Kff）保留。
	auto MakeAntiWindupGains = [](const FDronePidGains& Base, bool bSaturatedPos, bool bSaturatedNeg, float RateError) -> FDronePidGains
	{
		FDronePidGains G = Base;
		// 仅当误差方向与饱和方向一致时禁积分（PX4：saturated_positive → error=min(error,0)）
		if ((bSaturatedPos && RateError > 0.0f) || (bSaturatedNeg && RateError < 0.0f))
		{
			G.Ki = 0.0f;
		}
		return G;
	};

	const float RollError  = DesiredBodyRatesDegreesPerSec.X - CurrentBodyRates.X;
	const float PitchError = DesiredBodyRatesDegreesPerSec.Y - CurrentBodyRates.Y;
	const float YawError   = DesiredBodyRatesDegreesPerSec.Z - CurrentBodyRates.Z;

	const FDronePidGains RollGains  = MakeAntiWindupGains(Context.Config.Controller.Attitude.RateGains.Roll,  Context.AllocationFeedback.bSaturatedPositive[0], Context.AllocationFeedback.bSaturatedNegative[0], RollError);
	const FDronePidGains PitchGains = MakeAntiWindupGains(Context.Config.Controller.Attitude.RateGains.Pitch, Context.AllocationFeedback.bSaturatedPositive[1], Context.AllocationFeedback.bSaturatedNegative[1], PitchError);
	const FDronePidGains YawGains   = MakeAntiWindupGains(Context.Config.Controller.Attitude.RateGains.Yaw,   Context.AllocationFeedback.bSaturatedPositive[2], Context.AllocationFeedback.bSaturatedNegative[2], YawError);

	// u = Kp·(ω_des − ω) + Ki·∫ + Kd·d(ω)/dt + Kff·rate_ff
	return FVector(
		PidStates.Rate.Roll.UpdateFromMeasurement(DesiredBodyRatesDegreesPerSec.X, CurrentBodyRates.X, DeltaSeconds, RollGains, RateFeedForwardDegPerSec.X),
		PidStates.Rate.Pitch.UpdateFromMeasurement(DesiredBodyRatesDegreesPerSec.Y, CurrentBodyRates.Y, DeltaSeconds, PitchGains, RateFeedForwardDegPerSec.Y),
		PidStates.Rate.Yaw.UpdateFromMeasurement(DesiredBodyRatesDegreesPerSec.Z, CurrentBodyRates.Z, DeltaSeconds, YawGains, RateFeedForwardDegPerSec.Z));
}


FVector FFlightControlSolver::ComputeDesiredHorizontalVelocity(const FFlightControlSolverContext& Context, const FDronePilotInput& PilotInput) const
{
	const FRotator FlatYawRotation(0.0f, Context.Runtime.EstimatedState.State.AttitudeDegrees.Yaw, 0.0f);
	const FVector ForwardFlat = FRotationMatrix(FlatYawRotation).GetUnitAxis(EAxis::X);
	const FVector RightFlat = FRotationMatrix(FlatYawRotation).GetUnitAxis(EAxis::Y);
	const float MaxSpeed = Context.Config.Controller.Limits.MaxHorizontalSpeedCmPerSec;
	const FVector DesiredVelocity = ForwardFlat * (PilotInput.Pitch * MaxSpeed) + RightFlat * (PilotInput.Roll * MaxSpeed);
	return FVector(DesiredVelocity.X, DesiredVelocity.Y, 0.0f);
}


FVector FFlightControlSolver::ComputeDesiredHorizontalAcceleration(FFlightControlSolverContext& Context, const FDronePilotInput& PilotInput, float DeltaSeconds)
{
	const FVector CurrentPosition = Context.Runtime.EstimatedState.State.PositionCm;
	const FVector CurrentVelocity = Context.Runtime.EstimatedState.State.VelocityCmPerSec;

	if (!Context.ModeCapabilities.CanUsePositionControl && !Context.ModeCapabilities.CanUseVelocityControl)
	{
		// 无速度/位置控制能力时直接返回零加速度
		PidStates.Velocity.X.Reset(); PidStates.Velocity.Y.Reset();
		return FVector::ZeroVector;
	}

	// ======================================================================
	// Autopilot 注入路径
	// ======================================================================
	if (Context.bUseAutopilotSetpoint && Context.AutopilotInjection.bValid)
	{
		const FAutopilotInjection& AI = Context.AutopilotInjection;

		FVector DesiredVelocity = FVector::ZeroVector;
		FVector DesiredAcceleration = FVector::ZeroVector;

		if (Context.ModeCapabilities.CanUsePositionControl)
		{
			// 位置环：设定值 = PositionSetpointCm.XY，前馈 = VelocitySetpointCmPerSec.XY
			DesiredVelocity = FVector(
				PidStates.Position.X.UpdateFromMeasurement(AI.PositionSetpointCm.X, CurrentPosition.X, DeltaSeconds, Context.Config.Controller.Position.PositionGains.X, AI.VelocitySetpointCmPerSec.X),
				PidStates.Position.Y.UpdateFromMeasurement(AI.PositionSetpointCm.Y, CurrentPosition.Y, DeltaSeconds, Context.Config.Controller.Position.PositionGains.Y, AI.VelocitySetpointCmPerSec.Y),
				0.0f);

			Context.Runtime.ControlOutput.Targets.Position.bEnabled = true;
			Context.Runtime.ControlOutput.Targets.Position.PositionCm = FVector(AI.PositionSetpointCm.X, AI.PositionSetpointCm.Y, AI.AltitudeSetpointCm);
		}
		else
		{
			DesiredVelocity = AI.VelocitySetpointCmPerSec;
		}

		// 速度限幅
		DesiredVelocity.Z = 0.0f;
		const float MaxHSpeed = Context.Config.Controller.Limits.MaxHorizontalSpeedCmPerSec;
		const FVector2D DV2D(DesiredVelocity.X, DesiredVelocity.Y);
		if (DV2D.SizeSquared() > FMath::Square(MaxHSpeed))
		{
			const FVector2D Clamped = DV2D.GetSafeNormal() * MaxHSpeed;
			DesiredVelocity.X = Clamped.X; DesiredVelocity.Y = Clamped.Y;
		}

		Context.Runtime.ControlOutput.Targets.Velocity.bEnabled = true;
		Context.Runtime.ControlOutput.Targets.Velocity.VelocityCmPerSec.X = DesiredVelocity.X;
		Context.Runtime.ControlOutput.Targets.Velocity.VelocityCmPerSec.Y = DesiredVelocity.Y;

		// 速度环：前馈 = AccelerationSetpointCmPerSecSq.XY
		DesiredAcceleration.X = PidStates.Velocity.X.UpdateFromMeasurement(
			DesiredVelocity.X, CurrentVelocity.X, DeltaSeconds, Context.Config.Controller.Position.VelocityGains.X, AI.AccelerationSetpointCmPerSecSq.X);
		DesiredAcceleration.Y = PidStates.Velocity.Y.UpdateFromMeasurement(
			DesiredVelocity.Y, CurrentVelocity.Y, DeltaSeconds, Context.Config.Controller.Position.VelocityGains.Y, AI.AccelerationSetpointCmPerSecSq.Y);

		// 加速度限幅
		const float MaxHAccel = Context.Config.Controller.Limits.MaxHorizontalAccelerationCmPerSecSq;
		const FVector2D DA2D(DesiredAcceleration.X, DesiredAcceleration.Y);
		if (DA2D.SizeSquared() > FMath::Square(MaxHAccel))
		{
			const FVector2D Clamped = DA2D.GetSafeNormal() * MaxHAccel;
			DesiredAcceleration.X = Clamped.X; DesiredAcceleration.Y = Clamped.Y;
		}

		return FVector(DesiredAcceleration.X, DesiredAcceleration.Y, 0.0f);
	}

	// ======================================================================
	// 手动摇杆路径（原有逻辑）
	// ======================================================================

	// 先计算摇杆对应的期望速度
	FVector DesiredVelocity = ComputeDesiredHorizontalVelocity(Context, PilotInput);

	// ---- 位置环（如果可用）----
	if (Context.ModeCapabilities.CanUsePositionControl)
	{
		// 判断是否有手动水平摇杆指令
		const bool bManualHorizontalCommand = FMath::Abs(PilotInput.Roll) > Context.Config.Input.HorizontalHoldStickDeadband
			|| FMath::Abs(PilotInput.Pitch) > Context.Config.Input.HorizontalHoldStickDeadband;

		if (!Context.Runtime.HoldTargets.bPositionHoldInitialized)
		{
			// 首次进入位置保持 → 锁定当前位置
			Context.Runtime.HoldTargets.HeldPositionCm = CurrentPosition;
			Context.Runtime.HoldTargets.bPositionHoldInitialized = true;
			PidStates.Position.X.Reset(); PidStates.Position.Y.Reset();
		}

		// 手动输入时 → 重新锚定保持点，让位置 PID 不与手动指令打架
		if (bManualHorizontalCommand)
		{
			Context.Runtime.HoldTargets.HeldPositionCm = CurrentPosition;
			PidStates.Position.X.Reset(); PidStates.Position.Y.Reset();
		}
		else
		{
			// 无手动输入 → 位置 PID 生成期望速度
			//   v_des = PID_pos(pos_held − pos_current)
			//   使用 UpdateFromMeasurement（导数对测量值），避免位置设定值跳变的 kick
			DesiredVelocity = FVector(
				PidStates.Position.X.UpdateFromMeasurement(Context.Runtime.HoldTargets.HeldPositionCm.X, CurrentPosition.X, DeltaSeconds, Context.Config.Controller.Position.PositionGains.X),
				PidStates.Position.Y.UpdateFromMeasurement(Context.Runtime.HoldTargets.HeldPositionCm.Y, CurrentPosition.Y, DeltaSeconds, Context.Config.Controller.Position.PositionGains.Y),
				0.0);
		}

		Context.Runtime.ControlOutput.Targets.Position.bEnabled = true;
		Context.Runtime.ControlOutput.Targets.Position.PositionCm = FVector(
			Context.Runtime.HoldTargets.HeldPositionCm.X, Context.Runtime.HoldTargets.HeldPositionCm.Y, Context.Runtime.HoldTargets.HeldAltitudeCm);
	}
	else
	{
		Context.Runtime.HoldTargets.bPositionHoldInitialized = false;
		PidStates.Position.X.Reset(); PidStates.Position.Y.Reset();
	}

	// ---- 速度限幅 ----
	DesiredVelocity.Z = 0.0f;
	const float MaxHorizontalSpeed = Context.Config.Controller.Limits.MaxHorizontalSpeedCmPerSec;
	const FVector2D DesiredVelocity2D(DesiredVelocity.X, DesiredVelocity.Y);
	if (DesiredVelocity2D.SizeSquared() > FMath::Square(MaxHorizontalSpeed))
	{
		const FVector2D ClampedVelocity = DesiredVelocity2D.GetSafeNormal() * MaxHorizontalSpeed;
		DesiredVelocity.X = ClampedVelocity.X; DesiredVelocity.Y = ClampedVelocity.Y;
	}

	Context.Runtime.ControlOutput.Targets.Velocity.bEnabled = true;
	Context.Runtime.ControlOutput.Targets.Velocity.VelocityCmPerSec.X = DesiredVelocity.X;
	Context.Runtime.ControlOutput.Targets.Velocity.VelocityCmPerSec.Y = DesiredVelocity.Y;

	// ---- 速度环 → 期望加速度 ----
	//   a_des = PID_vel(v_des − v_current)
	//   使用 UpdateFromMeasurement（导数对测量值），避免速度设定值跳变的 kick
	FVector DesiredAcceleration = FVector::ZeroVector;
	DesiredAcceleration.X = PidStates.Velocity.X.UpdateFromMeasurement(
		DesiredVelocity.X, CurrentVelocity.X, DeltaSeconds, Context.Config.Controller.Position.VelocityGains.X);
	DesiredAcceleration.Y = PidStates.Velocity.Y.UpdateFromMeasurement(
		DesiredVelocity.Y, CurrentVelocity.Y, DeltaSeconds, Context.Config.Controller.Position.VelocityGains.Y);

	// ---- 加速度限幅 ----
	const float MaxHorizontalAcceleration = Context.Config.Controller.Limits.MaxHorizontalAccelerationCmPerSecSq;
	const FVector2D DesiredAcceleration2D(DesiredAcceleration.X, DesiredAcceleration.Y);
	if (DesiredAcceleration2D.SizeSquared() > FMath::Square(MaxHorizontalAcceleration))
	{
		const FVector2D ClampedAcceleration = DesiredAcceleration2D.GetSafeNormal() * MaxHorizontalAcceleration;
		DesiredAcceleration.X = ClampedAcceleration.X; DesiredAcceleration.Y = ClampedAcceleration.Y;
	}

	return FVector(DesiredAcceleration.X, DesiredAcceleration.Y, 0.0f);
}


float FFlightControlSolver::MapCenteredThrottleToCollective(const FFlightControlSolverContext& Context, float ThrottleInput) const
{
	const float HoverCollective = Context.Config.Controller.Limits.HoverCollectiveCommand;
	const float MinCollective = Context.Config.Controller.Limits.MinCollectiveCommand;
	const float MaxCollective = Context.Config.Controller.Limits.MaxCollectiveCommand;
	const float ClampedThrottle = FMath::Clamp(ThrottleInput, -1.0f, 1.0f);

	if (ClampedThrottle >= 0.0f)
		return FMath::Lerp(HoverCollective, MaxCollective, ClampedThrottle);
	else
		return FMath::Lerp(HoverCollective, MinCollective, -ClampedThrottle);
}





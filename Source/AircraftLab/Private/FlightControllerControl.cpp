#include "FlightControllerRuntimeObjects.h"

#include "Math/RotationMatrix.h"

FVector FlightControlDynamics::ComputeLinearDampingFeedForward(
	const FVector& DesiredVelocityCmPerSec, float LinearDampingPerSecond, float Scale)
{
	const float EffectiveDamping = FMath::Max(LinearDampingPerSecond, 0.0f) * FMath::Max(Scale, 0.0f);
	return FVector(DesiredVelocityCmPerSec.X * EffectiveDamping,
		DesiredVelocityCmPerSec.Y * EffectiveDamping, 0.0f);
}

float FlightControlDynamics::ComputeVerticalDampingCollectiveFeedForward(
	float DesiredVerticalVelocityCmPerSec, float LinearDampingPerSecond,
	float GravityCmPerSecSq, float HoverCollective, float Scale)
{
	const float Gravity = FMath::Max(GravityCmPerSecSq, UE_SMALL_NUMBER);
	const float DampingAcceleration = DesiredVerticalVelocityCmPerSec
		* FMath::Max(LinearDampingPerSecond, 0.0f) * FMath::Max(Scale, 0.0f);
	return FMath::Max(HoverCollective, 0.0f) * DampingAcceleration / Gravity;
}

FVector FlightControlDynamics::ComputeAngularDampingFeedForward(
	const FVector& DesiredBodyRatesDegPerSec, float AngularDampingPerSecond,
	const FVector& InertiaDiagonalKgM2, const FVector& PositiveTorqueAuthorityNm,
	const FVector& NegativeTorqueAuthorityNm, float Scale)
{
	const float EffectiveDamping = FMath::Max(AngularDampingPerSecond, 0.0f)
		* FMath::Max(Scale, 0.0f);
	const FVector DesiredBodyRatesRadPerSec = DesiredBodyRatesDegPerSec * (PI / 180.0f);
	const FVector RequiredTorqueNm = DesiredBodyRatesRadPerSec
		* InertiaDiagonalKgM2.ComponentMax(FVector::ZeroVector) * EffectiveDamping;
	FVector Result = FVector::ZeroVector;
	for (int32 Axis = 0; Axis < 3; ++Axis)
	{
		const float Authority = RequiredTorqueNm[Axis] >= 0.0f
			? PositiveTorqueAuthorityNm[Axis] : NegativeTorqueAuthorityNm[Axis];
		if (Authority > UE_SMALL_NUMBER)
		{
			Result[Axis] = RequiredTorqueNm[Axis] / Authority;
		}
	}
	return Result;
}

FlightControlDynamics::FDampingAwareHorizontalLimits
FlightControlDynamics::ComputeDampingAwareHorizontalLimits(
	float RequestedMaxSpeedCmPerSec, float PhysicalMaxAccelerationCmPerSecSq,
	float LinearDampingPerSecond, float ReserveFraction)
{
	FDampingAwareHorizontalLimits Result;
	const float RequestedSpeed = FMath::Max(RequestedMaxSpeedCmPerSec, 0.0f);
	const float PhysicalAcceleration = FMath::Max(PhysicalMaxAccelerationCmPerSecSq, 0.0f);
	const float Damping = FMath::Max(LinearDampingPerSecond, 0.0f);
	const float Reserve = FMath::Clamp(ReserveFraction, 0.0f, 0.9f);
	Result.MaxSpeedCmPerSec = RequestedSpeed;
	if (Damping > UE_SMALL_NUMBER)
	{
		const float DampingLimitedSpeed = PhysicalAcceleration * (1.0f - Reserve) / Damping;
		Result.MaxSpeedCmPerSec = FMath::Min(Result.MaxSpeedCmPerSec, DampingLimitedSpeed);
	}
	Result.MaxTrajectoryAccelerationCmPerSecSq = FMath::Max(
		PhysicalAcceleration - Damping * Result.MaxSpeedCmPerSec,
		PhysicalAcceleration * Reserve);
	return Result;
}

float FFlightControlSolver::ComputeVerticalControl(FFlightControlSolverContext& Context, float DeltaSeconds, float& OutDesiredVerticalVelocity)
{
	const float MinCollective = Context.Config.Controller.Limits.MinCollectiveCommand;
	const float HoverCollective = Context.Config.Controller.Limits.HoverCollectiveCommand;
	const float MaxCollective = Context.Config.Controller.Limits.MaxCollectiveCommand;
	const float CurrentAltitude = Context.Runtime.EstimatedState.State.PositionCm.Z;
	const float CurrentVerticalVelocity = Context.Runtime.EstimatedState.State.VelocityCmPerSec.Z;
	const auto SlewVerticalVelocitySetpoint = [&](float DesiredVelocity)
	{
		const float MaxAcceleration = FMath::Max(
			Context.Config.Controller.Limits.MaxVerticalAccelerationCmPerSecSq, 0.0f);
		if (!bVerticalVelocitySetpointInitialized)
		{
			LastDesiredVerticalVelocityCmPerSec = CurrentVerticalVelocity;
			bVerticalVelocitySetpointInitialized = true;
		}
		if (MaxAcceleration <= UE_SMALL_NUMBER)
		{
			LastDesiredVerticalVelocityCmPerSec = DesiredVelocity;
		}
		else
		{
			LastDesiredVerticalVelocityCmPerSec = FMath::FInterpConstantTo(
				LastDesiredVerticalVelocityCmPerSec, DesiredVelocity,
				DeltaSeconds, MaxAcceleration);
		}
		return LastDesiredVerticalVelocityCmPerSec;
	};

	if (!Context.ModeCapabilities.CanHoldAltitude)
	{
		// ---- 路径 A：无高度保持 ----
		// 重置 PID 状态，避免残留积分项
		Context.Runtime.HoldTargets.bAltitudeHoldInitialized = false;
		PidStates.Altitude.Reset();
		PidStates.VerticalVelocity.Reset();
		bVerticalVelocitySetpointInitialized = false;
		LastVerticalDampingCollectiveFeedForward = 0.0f;
		// 油门杆 → 垂直速度（线性映射）
		OutDesiredVerticalVelocity = Context.MovementIntent.DesiredVelocityCmPerSec.Z;
		// 油门杆 → 总距（悬停点为中心的线性映射）
		const float NormalizedVerticalCommand = OutDesiredVerticalVelocity >= 0.0f
			? OutDesiredVerticalVelocity / FMath::Max(Context.Config.Controller.Limits.MaxClimbRateCmPerSec, UE_SMALL_NUMBER)
			: OutDesiredVerticalVelocity / FMath::Max(Context.Config.Controller.Limits.MaxDescentRateCmPerSec, UE_SMALL_NUMBER);
		return MapCenteredThrottleToCollective(Context, NormalizedVerticalCommand);
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
		OutDesiredVerticalVelocity = SlewVerticalVelocitySetpoint(OutDesiredVerticalVelocity);
		const FDroneAltitudeControllerConfig& AltitudeConfig = Context.Config.Controller.Altitude;
		LastVerticalDampingCollectiveFeedForward =
			FlightControlDynamics::ComputeVerticalDampingCollectiveFeedForward(
				OutDesiredVerticalVelocity, Context.PhysicsCache.LinearDampingPerSecond,
				Context.PhysicsCache.GravityMagnitudeCmPerSecSq,
				Context.Config.Controller.Limits.HoverCollectiveCommand,
				AltitudeConfig.VerticalDampingFeedForwardScale);
		// 垂直速度内环；轨迹推力前馈作为基准，阻尼前馈补偿稳态阻力。
		const float CollectiveOffset = PidStates.VerticalVelocity.UpdateFromMeasurement(
			OutDesiredVerticalVelocity, CurrentVerticalVelocity, DeltaSeconds,
			AltitudeConfig.VerticalVelocityGains.ToRuntimeGains());
		// 推力前馈作总距基准（含重力补偿），替代 HoverCollective
		return FMath::Clamp(AI.ThrustFeedForward + LastVerticalDampingCollectiveFeedForward
			+ CollectiveOffset, MinCollective, MaxCollective);
	}

	// ---- 路径 B（手动）：高度保持 ----
	// RTH/AutoLand 内联已删除，由 BehaviorPlanner 经 TrajectoryGenerator 驱动
	{
		// 油门杆在死区外 → 手动爬升/下降率，重新锚定高度
		const float RequestedVerticalVelocity = Context.MovementIntent.DesiredVelocityCmPerSec.Z;
		if (FMath::Abs(RequestedVerticalVelocity) > UE_SMALL_NUMBER)
		{
			// 将死区外的输入线性映射到 [0,1]
			// 根据方向选择最大速率
			OutDesiredVerticalVelocity = RequestedVerticalVelocity;
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
	OutDesiredVerticalVelocity = SlewVerticalVelocitySetpoint(OutDesiredVerticalVelocity);
	// PID_vz: Δc = Kp·(v_z_des − v_z) + Ki·∫(v_z_des − v_z)dt + Kd·d(v_z_des − v_z)/dt
	// 输出 Δc 是总距偏移量，加在悬停点上
	const FDroneAltitudeControllerConfig& AltitudeConfig = Context.Config.Controller.Altitude;
	LastVerticalDampingCollectiveFeedForward =
		FlightControlDynamics::ComputeVerticalDampingCollectiveFeedForward(
			OutDesiredVerticalVelocity, Context.PhysicsCache.LinearDampingPerSecond,
			Context.PhysicsCache.GravityMagnitudeCmPerSecSq, HoverCollective,
			AltitudeConfig.VerticalDampingFeedForwardScale);
	const float CollectiveOffset = PidStates.VerticalVelocity.UpdateFromMeasurement(
		OutDesiredVerticalVelocity, CurrentVerticalVelocity, DeltaSeconds,
		Context.Config.Controller.Altitude.VerticalVelocityGains.ToRuntimeGains());
	// 最终总距 = 悬停总距 + PID偏移，限制在 [Min, Max]
	return FMath::Clamp(HoverCollective + LastVerticalDampingCollectiveFeedForward
		+ CollectiveOffset, MinCollective, MaxCollective);
}


FRotator FFlightControlSolver::ComputeDesiredAttitude(FFlightControlSolverContext& Context, float DeltaSeconds)
{
	if (!Context.ModeCapabilities.CanUseVelocityControl)
	{
		// ---- 路径 A：摇杆直接映射 ----
		Context.Runtime.HoldTargets.bPositionHoldInitialized = false;
		PidStates.Position.X.Reset(); PidStates.Position.Y.Reset();
		PidStates.Velocity.X.Reset(); PidStates.Velocity.Y.Reset();
		return Context.MovementIntent.DesiredAttitudeDegrees;
	}

	// ---- 路径 B：速度/位置 PID → 悬停倾斜方程 ----
	const FVector DesiredHorizontalAcceleration = ComputeDesiredHorizontalAcceleration(Context, DeltaSeconds);
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


FFlightControlYawSetpoint FFlightControlSolver::ComputeYawSetpoint(FFlightControlSolverContext& Context)
{
	FFlightControlYawSetpoint Result;
	const float CurrentYawDegrees = Context.Runtime.EstimatedState.State.AttitudeDegrees.Yaw;
	Result.TargetYawDegrees = CurrentYawDegrees;
	Result.MaxRateDegPerSec = Context.Config.Controller.Limits.MaxYawRateDegreesPerSec;

	// ---- 路径 C：Autopilot 注入 ----
	if (Context.bUseAutopilotSetpoint && Context.AutopilotInjection.bValid)
	{
		const FAutopilotInjection& AI = Context.AutopilotInjection;
		const float IntentYawRateLimit = AI.YawRateLimitDegPerSec > UE_SMALL_NUMBER
			? AI.YawRateLimitDegPerSec
			: Context.Config.Controller.Limits.MaxYawRateDegreesPerSec;
		Result.MaxRateDegPerSec = FMath::Min(
			Context.Config.Controller.Limits.MaxYawRateDegreesPerSec,
			IntentYawRateLimit);
		Result.TargetYawDegrees = FRotator::NormalizeAxis(AI.YawSetpointDegrees);
		Result.FeedForwardRateDegPerSec = FMath::Clamp(
			AI.YawRateSetpointDegPerSec, -Result.MaxRateDegPerSec, Result.MaxRateDegPerSec);
		return Result;
	}

	// ---- 手动路径 ----
	// 手动偏航角速率
	const float ManualYawRate = Context.MovementIntent.DesiredYawRateDegPerSec;

	if (!Context.ModeCapabilities.CanHoldYaw)
	{
		// 无航向保持：目标姿态使用当前航向，摇杆只作为角速度前馈。
		Context.Runtime.HoldTargets.bYawHoldInitialized = false;
		Result.FeedForwardRateDegPerSec = FMath::Clamp(
			ManualYawRate, -Result.MaxRateDegPerSec, Result.MaxRateDegPerSec);
		return Result;
	}

	// 摇杆超出死区 → 手动偏航率，同时重新锁定航向
	if (FMath::Abs(ManualYawRate) > UE_SMALL_NUMBER)
	{
		Context.Runtime.HoldTargets.HeldYawDegrees = Context.Runtime.EstimatedState.State.AttitudeDegrees.Yaw;
		Context.Runtime.HoldTargets.bYawHoldInitialized = true;
		Result.TargetYawDegrees = Context.Runtime.HoldTargets.HeldYawDegrees;
		Result.FeedForwardRateDegPerSec = FMath::Clamp(
			ManualYawRate, -Result.MaxRateDegPerSec, Result.MaxRateDegPerSec);
		return Result;
	}

	// 初始化锁定航向
	if (!Context.Runtime.HoldTargets.bYawHoldInitialized)
	{
		Context.Runtime.HoldTargets.HeldYawDegrees = Context.Runtime.EstimatedState.State.AttitudeDegrees.Yaw;
		Context.Runtime.HoldTargets.bYawHoldInitialized = true;
	}

	Result.TargetYawDegrees = FRotator::NormalizeAxis(Context.Runtime.HoldTargets.HeldYawDegrees);
	return Result;
}


FVector FFlightControlSolver::ComputeDesiredBodyRates(FFlightControlSolverContext& Context,
	const FRotator& DesiredAttitude, const FFlightControlYawSetpoint& YawSetpoint, float DeltaSeconds)
{
	// Acro/Manual 模式的默认值：摇杆直通
	float DesiredRollRate = Context.MovementIntent.DesiredBodyRatesDegPerSec.X;
	float DesiredPitchRate = Context.MovementIntent.DesiredBodyRatesDegPerSec.Y;
	float DesiredYawRate = YawSetpoint.FeedForwardRateDegPerSec;
	// 角速度前馈由姿态参考模型导数产生。
	float RollRateFF = 0.0f;
	float PitchRateFF = 0.0f;

	// 非角速度直通模式统一使用四元数姿态误差。
	if (Context.Runtime.AttitudeMode != EDroneAttitudeMode::Acro && Context.Runtime.AttitudeMode != EDroneAttitudeMode::Manual)
	{
		// 2 阶临界阻尼参考模型（对标 PX4 AttitudeControl.cpp）。
		// 对期望 Roll/Pitch 设定值做平滑：ẍ + 2ω·ẋ + ω²·(x − x_sp) = 0，ζ=1 临界阻尼。
		// Roll/Pitch 角度仅作为可读命令参数；姿态误差不在欧拉角空间计算。
		const FDroneAttitudeControllerConfig& AttCfg = Context.Config.Controller.Attitude;
		float SmoothedRoll = DesiredAttitude.Roll;
		float SmoothedPitch = DesiredAttitude.Pitch;

		if (AttCfg.bEnableAttitudeRefModel)
		{
			const float Omega = FMath::Max(AttCfg.RefModelNaturalFrequency, UE_SMALL_NUMBER);
			const float FFLimit = AttCfg.RefModelRateFFLimitDegPerSec;
			// ZOH 半隐式离散积分：
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

		// 直接使用 Chaos 刚体四元数，避免由姿态显示角反算当前姿态。
		const FQuat QCur = Context.PhysicsCache.BodyTransform.GetRotation().GetNormalized();
		const FQuat QDes = FRotator(
			SmoothedPitch, YawSetpoint.TargetYawDegrees, SmoothedRoll).Quaternion();
		FQuat QErr = QCur.Inverse() * QDes;
		if (QErr.W < 0.0f)
		{
			QErr = FQuat(-QErr.X, -QErr.Y, -QErr.Z, -QErr.W);
		}
		QErr.Normalize();

		// 2*q_err.imag 近似机体系姿态误差（rad）。X/Y 取负以匹配飞控
		// Roll/Pitch 角速度符号约定；转换到 deg/s 后叠加参考模型前馈。
		DesiredRollRate = FMath::RadiansToDegrees(
			-2.0f * QErr.X * AttCfg.QuaternionAttitudeGains.Roll) + RollRateFF;
		DesiredPitchRate = FMath::RadiansToDegrees(
			-2.0f * QErr.Y * AttCfg.QuaternionAttitudeGains.Pitch) + PitchRateFF;
		DesiredYawRate += FMath::RadiansToDegrees(
			2.0f * QErr.Z * AttCfg.QuaternionAttitudeGains.Yaw
			* FMath::Clamp(AttCfg.YawWeight, 0.0f, 1.0f));
	}

	// 限幅到最大角速率
	DesiredRollRate = FMath::Clamp(DesiredRollRate, -Context.Config.Controller.Limits.MaxRollRateDegreesPerSec, Context.Config.Controller.Limits.MaxRollRateDegreesPerSec);
	DesiredPitchRate = FMath::Clamp(DesiredPitchRate, -Context.Config.Controller.Limits.MaxPitchRateDegreesPerSec, Context.Config.Controller.Limits.MaxPitchRateDegreesPerSec);
	DesiredYawRate = FMath::Clamp(DesiredYawRate,
		-YawSetpoint.MaxRateDegPerSec, YawSetpoint.MaxRateDegPerSec);
	return FVector(DesiredRollRate, DesiredPitchRate, DesiredYawRate);
}


FVector FFlightControlSolver::ComputeBodyTorqueCommand(FFlightControlSolverContext& Context, const FVector& DesiredBodyRatesDegreesPerSec, float DeltaSeconds)
{
	const FVector CurrentBodyRates = Context.Runtime.EstimatedState.State.AngularVelocityBodyDegreesPerSec;

	// ---- 第 4 批：分配饱和回传抗 windup（对标 PX4 rate_control.cpp:88-117）----
	// 上一帧 AllocateToRotors 算出的饱和标志（1 帧延迟，可接受）。
	// 当某轴正/负方向分配饱和（残差>0/<0）时，禁止该方向角速度误差继续累积积分，
	// 避免积分项在"物理上无法满足"的方向上无限增长。
	// 实现：复制该轴增益并把 Ki 置零（仅在饱和方向），其余反馈项保留。
	auto MakeAntiWindupGains = [](const FDroneFeedbackPidGains& Base, bool bSaturatedPos, bool bSaturatedNeg, float RateError) -> FDronePidGains
	{
		FDronePidGains G = Base.ToRuntimeGains();
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

	FDronePidGains RollGains  = MakeAntiWindupGains(Context.Config.Controller.Attitude.RateGains.Roll,  Context.AllocationFeedback.bSaturatedPositive[0], Context.AllocationFeedback.bSaturatedNegative[0], RollError);
	FDronePidGains PitchGains = MakeAntiWindupGains(Context.Config.Controller.Attitude.RateGains.Pitch, Context.AllocationFeedback.bSaturatedPositive[1], Context.AllocationFeedback.bSaturatedNegative[1], PitchError);
	FDronePidGains YawGains   = MakeAntiWindupGains(Context.Config.Controller.Attitude.RateGains.Yaw,   Context.AllocationFeedback.bSaturatedPositive[2], Context.AllocationFeedback.bSaturatedNegative[2], YawError);

	const FDroneAttitudeControllerConfig& AttitudeConfig = Context.Config.Controller.Attitude;
	if (AttitudeConfig.AngularDampingFeedForwardScale > UE_SMALL_NUMBER)
	{
		const FVector PositiveAuthority(
			Context.AllocationFeedback.Cache.PositiveTorqueAuthority[0],
			Context.AllocationFeedback.Cache.PositiveTorqueAuthority[1],
			Context.AllocationFeedback.Cache.PositiveTorqueAuthority[2]);
		const FVector NegativeAuthority(
			Context.AllocationFeedback.Cache.NegativeTorqueAuthority[0],
			Context.AllocationFeedback.Cache.NegativeTorqueAuthority[1],
			Context.AllocationFeedback.Cache.NegativeTorqueAuthority[2]);
		LastAngularDampingFeedForward = FlightControlDynamics::ComputeAngularDampingFeedForward(
			DesiredBodyRatesDegreesPerSec, Context.PhysicsCache.AngularDampingPerSecond,
			Context.PhysicsCache.InertiaDiagonalKgM2, PositiveAuthority, NegativeAuthority,
			AttitudeConfig.AngularDampingFeedForwardScale);
	}
	else
	{
		LastAngularDampingFeedForward = FVector::ZeroVector;
	}
	// Damping FF 已是归一化轴指令；借用 PID Kff 通道可统一限幅和 anti-windup。
	RollGains.Kff = 1.0f;
	PitchGains.Kff = 1.0f;
	YawGains.Kff = 1.0f;

	// u = Kp·(ω_des − ω) + Ki·∫ + Kd·d(ω)/dt + normalized_damping_ff
	return FVector(
		PidStates.Rate.Roll.UpdateFromMeasurement(DesiredBodyRatesDegreesPerSec.X, CurrentBodyRates.X, DeltaSeconds, RollGains, LastAngularDampingFeedForward.X),
		PidStates.Rate.Pitch.UpdateFromMeasurement(DesiredBodyRatesDegreesPerSec.Y, CurrentBodyRates.Y, DeltaSeconds, PitchGains, LastAngularDampingFeedForward.Y),
		PidStates.Rate.Yaw.UpdateFromMeasurement(DesiredBodyRatesDegreesPerSec.Z, CurrentBodyRates.Z, DeltaSeconds, YawGains, LastAngularDampingFeedForward.Z));
}


FVector FFlightControlSolver::ComputeDesiredHorizontalVelocity(const FFlightControlSolverContext& Context) const
{
	const FVector DesiredVelocity = Context.MovementIntent.DesiredVelocityCmPerSec;
	return FVector(DesiredVelocity.X, DesiredVelocity.Y, 0.0f);
}


FVector FFlightControlSolver::ComputeVelocityPidAcceleration(
	FFlightControlSolverContext& Context, const FVector& DesiredVelocityCmPerSec,
	const FVector& TrajectoryAccelerationFeedForwardCmPerSecSq, float DeltaSeconds)
{
	const FDronePositionControllerConfig& PositionConfig = Context.Config.Controller.Position;
	const FVector CurrentVelocity = Context.Runtime.EstimatedState.State.VelocityCmPerSec;
	const FVector DragFeedForward =
		FlightControlDynamics::ComputeLinearDampingFeedForward(
			DesiredVelocityCmPerSec, Context.PhysicsCache.LinearDampingPerSecond,
			PositionConfig.LinearDampingFeedForwardScale);
	const FVector TrajectoryFeedForward(
		TrajectoryAccelerationFeedForwardCmPerSecSq.X,
		TrajectoryAccelerationFeedForwardCmPerSecSq.Y,
		0.0f);
	const FVector TotalFeedForward = DragFeedForward + TrajectoryFeedForward;

	FVector DesiredAcceleration(
		PidStates.Velocity.X.UpdateFromMeasurement(
			DesiredVelocityCmPerSec.X, CurrentVelocity.X, DeltaSeconds,
			PositionConfig.VelocityGains.X, TotalFeedForward.X),
		PidStates.Velocity.Y.UpdateFromMeasurement(
			DesiredVelocityCmPerSec.Y, CurrentVelocity.Y, DeltaSeconds,
			PositionConfig.VelocityGains.Y, TotalFeedForward.Y),
		0.0f);

	const FDroneControlLimits& ControlLimits = Context.Config.Controller.Limits;
	const float TiltLimitedAcceleration = Context.PhysicsCache.GravityMagnitudeCmPerSecSq
		* FMath::Tan(FMath::DegreesToRadians(ControlLimits.MaxTiltAngleDegrees));
	const float MaxHorizontalAcceleration = FMath::Min(
		ControlLimits.MaxHorizontalAccelerationCmPerSecSq, TiltLimitedAcceleration);
	const FVector2D HorizontalAcceleration(DesiredAcceleration.X, DesiredAcceleration.Y);
	if (HorizontalAcceleration.SizeSquared() > FMath::Square(MaxHorizontalAcceleration))
	{
		const FVector2D Clamped = HorizontalAcceleration.GetSafeNormal() * MaxHorizontalAcceleration;
		DesiredAcceleration.X = Clamped.X;
		DesiredAcceleration.Y = Clamped.Y;
	}

	LastDesiredHorizontalVelocityCmPerSec = FVector(DesiredVelocityCmPerSec.X, DesiredVelocityCmPerSec.Y, 0.0f);
	LastVelocityDragFeedForwardCmPerSecSq = DragFeedForward;
	LastTrajectoryAccelerationFeedForwardCmPerSecSq = TrajectoryFeedForward;
	LastDesiredHorizontalAccelerationCmPerSecSq = DesiredAcceleration;
	return DesiredAcceleration;
}


FVector FFlightControlSolver::ComputeDesiredHorizontalAcceleration(FFlightControlSolverContext& Context, float DeltaSeconds)
{
	const FVector CurrentPosition = Context.Runtime.EstimatedState.State.PositionCm;
	const FDroneControlLimits& ControlLimits = Context.Config.Controller.Limits;
	const float TiltLimitedAcceleration = Context.PhysicsCache.GravityMagnitudeCmPerSecSq
		* FMath::Tan(FMath::DegreesToRadians(ControlLimits.MaxTiltAngleDegrees));
	const float PhysicalHorizontalAcceleration = FMath::Min(
		ControlLimits.MaxHorizontalAccelerationCmPerSecSq, TiltLimitedAcceleration);
	const FlightControlDynamics::FDampingAwareHorizontalLimits DampingAwareLimits =
		FlightControlDynamics::ComputeDampingAwareHorizontalLimits(
			ControlLimits.MaxHorizontalSpeedCmPerSec, PhysicalHorizontalAcceleration,
			Context.PhysicsCache.LinearDampingPerSecond,
			Context.Config.Controller.Position.DampingAccelerationReserveFraction);
	const float ReachableHorizontalSpeed = DampingAwareLimits.MaxSpeedCmPerSec;

	if (!Context.ModeCapabilities.CanUsePositionControl && !Context.ModeCapabilities.CanUseVelocityControl)
	{
		// 无速度/位置控制能力时直接返回零加速度
		PidStates.Velocity.X.Reset(); PidStates.Velocity.Y.Reset();
		LastDesiredHorizontalVelocityCmPerSec = FVector::ZeroVector;
		LastVelocityDragFeedForwardCmPerSecSq = FVector::ZeroVector;
		LastTrajectoryAccelerationFeedForwardCmPerSecSq = FVector::ZeroVector;
		LastDesiredHorizontalAccelerationCmPerSecSq = FVector::ZeroVector;
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
		const float MaxHSpeed = ReachableHorizontalSpeed;
		const FVector2D DV2D(DesiredVelocity.X, DesiredVelocity.Y);
		if (DV2D.SizeSquared() > FMath::Square(MaxHSpeed))
		{
			const FVector2D Clamped = DV2D.GetSafeNormal() * MaxHSpeed;
			DesiredVelocity.X = Clamped.X; DesiredVelocity.Y = Clamped.Y;
		}

		Context.Runtime.ControlOutput.Targets.Velocity.bEnabled = true;
		Context.Runtime.ControlOutput.Targets.Velocity.VelocityCmPerSec.X = DesiredVelocity.X;
		Context.Runtime.ControlOutput.Targets.Velocity.VelocityCmPerSec.Y = DesiredVelocity.Y;

		// 速度 PID + 轨迹加速度前馈 + 维持目标速度所需的线性阻尼前馈。
		DesiredAcceleration = ComputeVelocityPidAcceleration(
			Context, DesiredVelocity, AI.AccelerationSetpointCmPerSecSq, DeltaSeconds);
		return DesiredAcceleration;
	}

	// ======================================================================
	// 手动摇杆路径（原有逻辑）
	// ======================================================================

	// 先计算摇杆对应的期望速度
	FVector DesiredVelocity = ComputeDesiredHorizontalVelocity(Context);

	// ---- 位置环（如果可用）----
	if (Context.ModeCapabilities.CanUsePositionControl)
	{
		// 判断是否有手动水平摇杆指令
		const bool bManualHorizontalCommand = !FVector2D(
			Context.MovementIntent.DesiredVelocityCmPerSec.X,
			Context.MovementIntent.DesiredVelocityCmPerSec.Y).IsNearlyZero();

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
			Context.Runtime.HoldTargets.bHorizontalBrakeBeforeHold = true;
			PidStates.Position.X.Reset(); PidStates.Position.Y.Reset();
		}
		else if (Context.Runtime.HoldTargets.bHorizontalBrakeBeforeHold)
		{
			// Stick release is a velocity-to-zero transition, not an immediate
			// request to return to the release point. Keep moving the anchor with
			// the aircraft while braking, then latch where it actually stops.
			Context.Runtime.HoldTargets.HeldPositionCm = CurrentPosition;
			PidStates.Position.X.Reset(); PidStates.Position.Y.Reset();
			const float HorizontalSpeed = FVector2D(
				Context.Runtime.EstimatedState.State.VelocityCmPerSec.X,
				Context.Runtime.EstimatedState.State.VelocityCmPerSec.Y).Size();
			if (HorizontalSpeed <= Context.Config.Input.HorizontalBrakeToHoldSpeedCmPerSec)
			{
				Context.Runtime.HoldTargets.bHorizontalBrakeBeforeHold = false;
			}
			DesiredVelocity = FVector::ZeroVector;
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
	const float MaxHorizontalSpeed = ReachableHorizontalSpeed;
	const FVector2D DesiredVelocity2D(DesiredVelocity.X, DesiredVelocity.Y);
	if (DesiredVelocity2D.SizeSquared() > FMath::Square(MaxHorizontalSpeed))
	{
		const FVector2D ClampedVelocity = DesiredVelocity2D.GetSafeNormal() * MaxHorizontalSpeed;
		DesiredVelocity.X = ClampedVelocity.X; DesiredVelocity.Y = ClampedVelocity.Y;
	}

	Context.Runtime.ControlOutput.Targets.Velocity.bEnabled = true;
	Context.Runtime.ControlOutput.Targets.Velocity.VelocityCmPerSec.X = DesiredVelocity.X;
	Context.Runtime.ControlOutput.Targets.Velocity.VelocityCmPerSec.Y = DesiredVelocity.Y;

	// ---- 速度 PID + 维持目标速度所需的线性阻尼前馈 ----
	return ComputeVelocityPidAcceleration(
		Context, DesiredVelocity, FVector::ZeroVector, DeltaSeconds);
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





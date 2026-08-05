// 对应 NxGame AircraftLab/Private/FlightControllerControl.cpp（逐函数对齐重写）。
// 配置来源由 NxGame 的嵌套 Profile 结构改为本模块的平铺 FAircraftFlightControllerRuntimeConfig。

#include "Aircraft/FlightControlSolver.h"

#include "Aircraft/ControlAllocator.h"
#include "Math/RotationMatrix.h"

namespace
{
	/**
	 * 从刚体四元数提取世界水平面中的机头方向。
	 * 正常姿态使用机体 Forward；其水平投影退化时，使用机体 Right 重建 Forward。
	 */
	FVector GetPlanarHeadingDirection(
		const FQuat& BodyRotation, const FAircraftFlightControllerRuntimeConfig& Config)
	{
		FVector Forward = BodyRotation.RotateVector(Config.GetForwardAxisBody());
		Forward.Z = 0.0f;
		if (Forward.Normalize())
		{
			return Forward;
		}

		FVector Right = BodyRotation.RotateVector(Config.GetRightAxisBody());
		Right.Z = 0.0f;
		if (Right.Normalize())
		{
			return FVector(Right.Y, -Right.X, 0.0f);
		}

		return FVector::ForwardVector;
	}

	float GetPlanarHeadingDegrees(
		const FQuat& BodyRotation, const FAircraftFlightControllerRuntimeConfig& Config)
	{
		const FVector Forward = GetPlanarHeadingDirection(BodyRotation, Config);
		return FMath::RadiansToDegrees(FMath::Atan2(Forward.Y, Forward.X));
	}

	/** 返回从当前水平航向转到目标航向的最短有符号角，单位为弧度。 */
	float ComputePlanarHeadingErrorRadians(
		const FQuat& BodyRotation, float TargetYawDegrees,
		const FAircraftFlightControllerRuntimeConfig& Config)
	{
		const FVector CurrentForward = GetPlanarHeadingDirection(BodyRotation, Config);
		const FQuat TargetHeadingRotation(
			FVector::UpVector, FMath::DegreesToRadians(TargetYawDegrees));
		const FVector TargetForward = TargetHeadingRotation.RotateVector(FVector::ForwardVector);
		return FMath::Atan2(
			FVector::CrossProduct(CurrentForward, TargetForward).Z,
			FVector::DotProduct(CurrentForward, TargetForward));
	}
}

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

float FAircraftFlightControlSolver::ComputeVerticalControl(FAircraftFlightControlSolverContext& Context, float DeltaSeconds, float& OutDesiredVerticalVelocity)
{
	const FAircraftFlightControllerRuntimeConfig& Config = Context.Config;
	const float MinCollective = Config.MinCollectiveCommand;
	const float HoverCollective = Config.HoverCollectiveCommand;
	const float MaxCollective = Config.MaxCollectiveCommand;
	const float CurrentAltitude = Context.Runtime.EstimatedState.State.PositionCm.Z;
	const float CurrentVerticalVelocity = Context.Runtime.EstimatedState.State.VelocityCmPerSec.Z;
	const auto SlewVerticalVelocitySetpoint = [&](float DesiredVelocity)
	{
		const float MaxAcceleration = FMath::Max(Config.MaxVerticalAccelerationCmPerSecSq, 0.0f);
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
		Context.Runtime.HoldTargets.bAltitudeHoldInitialized = false;
		PidStates.Altitude.Reset();
		PidStates.VerticalVelocity.Reset();
		bVerticalVelocitySetpointInitialized = false;
		LastVerticalDampingCollectiveFeedForward = 0.0f;
		// 油门杆 → 垂直速度（线性映射）
		OutDesiredVerticalVelocity = Context.ManualCommand.DesiredVelocityCmPerSec.Z;
		// 油门杆 → 总距（悬停点为中心的线性映射）
		const float NormalizedVerticalCommand = OutDesiredVerticalVelocity >= 0.0f
			? OutDesiredVerticalVelocity / FMath::Max(Config.MaxClimbRateCmPerSec, UE_SMALL_NUMBER)
			: OutDesiredVerticalVelocity / FMath::Max(Config.MaxDescentRateCmPerSec, UE_SMALL_NUMBER);
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
			Config.GetAltitudePidGains(), AI.VerticalVelocitySetpointCmPerSec);
		OutDesiredVerticalVelocity = FMath::Clamp(OutDesiredVerticalVelocity,
			-Config.MaxDescentRateCmPerSec, Config.MaxClimbRateCmPerSec);
		OutDesiredVerticalVelocity = SlewVerticalVelocitySetpoint(OutDesiredVerticalVelocity);
		LastVerticalDampingCollectiveFeedForward =
			FlightControlDynamics::ComputeVerticalDampingCollectiveFeedForward(
				OutDesiredVerticalVelocity, Context.PhysicsCache.LinearDampingPerSecond,
				Context.PhysicsCache.GravityMagnitudeCmPerSecSq,
				Config.HoverCollectiveCommand,
				Config.VerticalDampingFeedForwardScale);
		// 垂直速度内环；轨迹推力前馈作为基准，阻尼前馈补偿稳态阻力。
		const float CollectiveOffset = PidStates.VerticalVelocity.UpdateFromMeasurement(
			OutDesiredVerticalVelocity, CurrentVerticalVelocity, DeltaSeconds,
			Config.GetVerticalVelocityPidGains());
		// 推力前馈作总距基准（含重力补偿），替代 HoverCollective
		return FMath::Clamp(AI.ThrustFeedForward + LastVerticalDampingCollectiveFeedForward
			+ CollectiveOffset, MinCollective, MaxCollective);
	}

	// ---- 路径 B（手动）：高度保持 ----
	{
		// 油门杆在死区外 → 手动爬升/下降率，重新锚定高度
		const float RequestedVerticalVelocity = Context.ManualCommand.DesiredVelocityCmPerSec.Z;
		if (FMath::Abs(RequestedVerticalVelocity) > UE_SMALL_NUMBER)
		{
			OutDesiredVerticalVelocity = RequestedVerticalVelocity;
			// 重新锚定高度到当前位置（松手后将保持新高度）
			Context.Runtime.HoldTargets.HeldAltitudeCm = CurrentAltitude;
			PidStates.Altitude.Reset();
		}
		else
		{
			// 油门杆在死区内 → 高度 PID 保持锁定高度（导数对测量值，避免设定值跳变 kick）
			OutDesiredVerticalVelocity = PidStates.Altitude.UpdateFromMeasurement(
				Context.Runtime.HoldTargets.HeldAltitudeCm, CurrentAltitude, DeltaSeconds,
				Config.GetAltitudePidGains());
			OutDesiredVerticalVelocity = FMath::Clamp(OutDesiredVerticalVelocity,
				-Config.MaxDescentRateCmPerSec, Config.MaxClimbRateCmPerSec);
		}
	}

	// ---- 垂直速度内环（手动路径）----
	OutDesiredVerticalVelocity = SlewVerticalVelocitySetpoint(OutDesiredVerticalVelocity);
	// PID_vz 输出 Δc 是总距偏移量，加在悬停点上
	LastVerticalDampingCollectiveFeedForward =
		FlightControlDynamics::ComputeVerticalDampingCollectiveFeedForward(
			OutDesiredVerticalVelocity, Context.PhysicsCache.LinearDampingPerSecond,
			Context.PhysicsCache.GravityMagnitudeCmPerSecSq, HoverCollective,
			Config.VerticalDampingFeedForwardScale);
	const float CollectiveOffset = PidStates.VerticalVelocity.UpdateFromMeasurement(
		OutDesiredVerticalVelocity, CurrentVerticalVelocity, DeltaSeconds,
		Config.GetVerticalVelocityPidGains());
	// 最终总距 = 悬停总距 + PID偏移，限制在 [Min, Max]
	return FMath::Clamp(HoverCollective + LastVerticalDampingCollectiveFeedForward
		+ CollectiveOffset, MinCollective, MaxCollective);
}


FRotator FAircraftFlightControlSolver::ComputeDesiredAttitude(FAircraftFlightControlSolverContext& Context, float DeltaSeconds)
{
	const FAircraftFlightControllerRuntimeConfig& Config = Context.Config;
	if (!Context.ModeCapabilities.CanUseVelocityControl)
	{
		// ---- 路径 A：摇杆直接映射 ----
		Context.Runtime.HoldTargets.bPositionHoldInitialized = false;
		PidStates.Position.X.Reset(); PidStates.Position.Y.Reset();
		PidStates.Velocity.X.Reset(); PidStates.Velocity.Y.Reset();
		return Context.ManualCommand.DesiredAttitudeDegrees;
	}

	// ---- 路径 B：速度/位置 PID → 悬停倾斜方程 ----
	const FVector DesiredHorizontalAcceleration = ComputeDesiredHorizontalAcceleration(Context, DeltaSeconds);
	const float GravityMagnitude = Context.PhysicsCache.GravityMagnitudeCmPerSecSq;
	const float CurrentHeadingDegrees = GetPlanarHeadingDegrees(
		Context.PhysicsCache.BodyTransform.GetRotation().GetNormalized(), Config);

	// 构造仅含航向的"平面旋转"——提取机体前/右方向的水平投影
	const FRotator FlatYawRotation(0.0f, CurrentHeadingDegrees, 0.0f);
	const FVector ForwardFlat = FRotationMatrix(FlatYawRotation).GetUnitAxis(EAxis::X);
	const FVector RightFlat = FRotationMatrix(FlatYawRotation).GetUnitAxis(EAxis::Y);

	// 将期望加速度投影到机体前/右方向
	const float ForwardAcceleration = FVector::DotProduct(DesiredHorizontalAcceleration, ForwardFlat);
	const float RightAcceleration = FVector::DotProduct(DesiredHorizontalAcceleration, RightFlat);

	// 悬停倾斜方程：tan(θ) = a/g
	float DesiredPitchDegrees = -FMath::RadiansToDegrees(FMath::Atan2(ForwardAcceleration, GravityMagnitude));
	float DesiredRollDegrees = FMath::RadiansToDegrees(FMath::Atan2(RightAcceleration, GravityMagnitude));

	// Autopilot 协调转弯滚转叠加（叠加在悬停倾斜方程之上）
	if (Context.bUseAutopilotSetpoint && Context.AutopilotInjection.bValid)
	{
		DesiredRollDegrees += Context.AutopilotInjection.TurnRollDegrees;
	}

	// 限制最大倾角——超出此角度可能推力不足以抵消重力分量
	DesiredRollDegrees = FMath::Clamp(DesiredRollDegrees, -Config.MaxTiltAngleDegrees, Config.MaxTiltAngleDegrees);
	DesiredPitchDegrees = FMath::Clamp(DesiredPitchDegrees, -Config.MaxTiltAngleDegrees, Config.MaxTiltAngleDegrees);
	return FRotator(DesiredPitchDegrees, CurrentHeadingDegrees, DesiredRollDegrees);
}


FAircraftYawSetpoint FAircraftFlightControlSolver::ComputeYawSetpoint(FAircraftFlightControlSolverContext& Context)
{
	const FAircraftFlightControllerRuntimeConfig& Config = Context.Config;
	FAircraftYawSetpoint Result;
	// 航向保持初始化与下游航向误差必须使用同一份刚体四元数真值。
	const float CurrentYawDegrees = GetPlanarHeadingDegrees(
		Context.PhysicsCache.BodyTransform.GetRotation().GetNormalized(), Config);
	Result.TargetYawDegrees = CurrentYawDegrees;
	Result.MaxRateDegPerSec = Config.MaxYawRateDegreesPerSec;

	// ---- 路径 C：Autopilot 注入 ----
	if (Context.bUseAutopilotSetpoint && Context.AutopilotInjection.bValid)
	{
		const FAutopilotInjection& AI = Context.AutopilotInjection;
		const float IntentYawRateLimit = AI.YawRateLimitDegPerSec > UE_SMALL_NUMBER
			? AI.YawRateLimitDegPerSec
			: Config.MaxYawRateDegreesPerSec;
		Result.MaxRateDegPerSec = FMath::Min(Config.MaxYawRateDegreesPerSec, IntentYawRateLimit);
		Result.TargetYawDegrees = FRotator::NormalizeAxis(AI.YawSetpointDegrees);
		Result.FeedForwardRateDegPerSec = FMath::Clamp(
			AI.YawRateSetpointDegPerSec, -Result.MaxRateDegPerSec, Result.MaxRateDegPerSec);
		return Result;
	}

	// ---- 手动路径 ----
	const float ManualYawRate = Context.ManualCommand.DesiredYawRateDegPerSec;

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
		Context.Runtime.HoldTargets.HeldYawDegrees = CurrentYawDegrees;
		Context.Runtime.HoldTargets.bYawHoldInitialized = true;
		Result.TargetYawDegrees = Context.Runtime.HoldTargets.HeldYawDegrees;
		Result.FeedForwardRateDegPerSec = FMath::Clamp(
			ManualYawRate, -Result.MaxRateDegPerSec, Result.MaxRateDegPerSec);
		return Result;
	}

	// 初始化锁定航向
	if (!Context.Runtime.HoldTargets.bYawHoldInitialized)
	{
		Context.Runtime.HoldTargets.HeldYawDegrees = CurrentYawDegrees;
		Context.Runtime.HoldTargets.bYawHoldInitialized = true;
	}

	Result.TargetYawDegrees = FRotator::NormalizeAxis(Context.Runtime.HoldTargets.HeldYawDegrees);
	return Result;
}


FVector FAircraftFlightControlSolver::ComputeDesiredBodyRates(FAircraftFlightControlSolverContext& Context,
	const FRotator& DesiredAttitude, const FAircraftYawSetpoint& YawSetpoint, float DeltaSeconds)
{
	const FAircraftFlightControllerRuntimeConfig& Config = Context.Config;
	// Acro/Manual 模式的默认值：摇杆直通
	float DesiredRollRate = Context.ManualCommand.DesiredBodyRatesDegPerSec.X;
	float DesiredPitchRate = Context.ManualCommand.DesiredBodyRatesDegPerSec.Y;
	float DesiredYawRate = YawSetpoint.FeedForwardRateDegPerSec;
	// 角速度前馈由姿态参考模型导数产生。
	float RollRateFF = 0.0f;
	float PitchRateFF = 0.0f;

	// 非角速度直通模式统一使用四元数姿态误差。
	if (Context.Runtime.AttitudeMode != EAircraftAttitudeMode::Acro && Context.Runtime.AttitudeMode != EAircraftAttitudeMode::Manual)
	{
		// 2 阶临界阻尼参考模型（对标 PX4 AttitudeControl.cpp）。
		// ẍ + 2ω·ẋ + ω²·(x − x_sp) = 0，ζ=1 临界阻尼。
		float SmoothedRoll = DesiredAttitude.Roll;
		float SmoothedPitch = DesiredAttitude.Pitch;

		if (Config.bEnableAttitudeReferenceModel)
		{
			const float Omega = FMath::Max(Config.ReferenceModelNaturalFrequency, UE_SMALL_NUMBER);
			const float FFLimit = Config.ReferenceModelRateFeedForwardLimitDegPerSec;
			// ZOH 半隐式离散积分：
			//   v += ω²·(x_sp − x)·dt − 2ω·v·dt
			//   x += v·dt
			auto StepRefModel = [Omega, DeltaSeconds](FAircraftReferenceModelState& S, float Setpoint)
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

		// Roll/Pitch 命令是在当前机头航向坐标系中生成的，目标倾斜姿态必须继续使用当前航向；
		// 目标 Yaw 作为独立航向闭环处理，避免 Yaw 误差泄漏到 Roll/Pitch 通道。
		const FQuat QBody = Context.PhysicsCache.BodyTransform.GetRotation().GetNormalized();
		const FQuat QCur = Config.GetControlWorldRotation(QBody);
		const float CurrentHeadingDegrees = GetPlanarHeadingDegrees(QBody, Config);
		const FQuat QDes = FRotator(
			SmoothedPitch, CurrentHeadingDegrees, SmoothedRoll).Quaternion();
		FQuat QErr = QCur.Inverse() * QDes;
		if (QErr.W < 0.0f)
		{
			QErr = FQuat(-QErr.X, -QErr.Y, -QErr.Z, -QErr.W);
		}
		QErr.Normalize();

		// 2·q_err.imag 近似机体系姿态误差（rad）。X/Y 取负以匹配飞控
		// Roll/Pitch 角速度符号约定；转换到 deg/s 后叠加参考模型前馈。
		DesiredRollRate = FMath::RadiansToDegrees(
			-2.0f * QErr.X * Config.AttitudeGains.X) + RollRateFF;
		DesiredPitchRate = FMath::RadiansToDegrees(
			-2.0f * QErr.Y * Config.AttitudeGains.Y) + PitchRateFF;

		// 航向误差来自刚体四元数的水平机头方向；比例增益产生偏航角速度，前馈叠加。
		const float HeadingErrorRadians = ComputePlanarHeadingErrorRadians(
			QBody, YawSetpoint.TargetYawDegrees, Config);
		DesiredYawRate += FMath::RadiansToDegrees(
			HeadingErrorRadians * Config.AttitudeGains.Z);
	}

	// 限幅到最大角速率
	DesiredRollRate = FMath::Clamp(DesiredRollRate, -Config.MaxRollRateDegreesPerSec, Config.MaxRollRateDegreesPerSec);
	DesiredPitchRate = FMath::Clamp(DesiredPitchRate, -Config.MaxPitchRateDegreesPerSec, Config.MaxPitchRateDegreesPerSec);
	DesiredYawRate = FMath::Clamp(DesiredYawRate,
		-YawSetpoint.MaxRateDegPerSec, YawSetpoint.MaxRateDegPerSec);
	return FVector(DesiredRollRate, DesiredPitchRate, DesiredYawRate);
}


FVector FAircraftFlightControlSolver::ComputeBodyTorqueCommand(FAircraftFlightControlSolverContext& Context, const FVector& DesiredBodyRatesDegreesPerSec, float DeltaSeconds)
{
	const FAircraftFlightControllerRuntimeConfig& Config = Context.Config;
	const FVector CurrentBodyRates = Context.Runtime.EstimatedState.State.AngularVelocityBodyDegreesPerSec;

	// ---- 分配饱和回传抗 windup（对标 PX4 rate_control.cpp）----
	// 上一帧 Allocate 算出的饱和标志（1 帧延迟，可接受）：
	// 某轴正/负方向分配饱和时，禁止该方向角速度误差继续累积积分。
	auto MakeAntiWindupGains = [](FAircraftPidGains G, bool bSaturatedPos, bool bSaturatedNeg, float RateError) -> FAircraftPidGains
	{
		// 仅当误差方向与饱和方向一致时禁积分
		if ((bSaturatedPos && RateError > 0.0f) || (bSaturatedNeg && RateError < 0.0f))
		{
			G.Ki = 0.0f;
		}
		return G;
	};

	const float RollError  = DesiredBodyRatesDegreesPerSec.X - CurrentBodyRates.X;
	const float PitchError = DesiredBodyRatesDegreesPerSec.Y - CurrentBodyRates.Y;
	const float YawError   = DesiredBodyRatesDegreesPerSec.Z - CurrentBodyRates.Z;

	FAircraftPidGains RollGains  = MakeAntiWindupGains(Config.GetRatePidGains(0), Context.AllocationFeedback.bSaturatedPositive[0], Context.AllocationFeedback.bSaturatedNegative[0], RollError);
	FAircraftPidGains PitchGains = MakeAntiWindupGains(Config.GetRatePidGains(1), Context.AllocationFeedback.bSaturatedPositive[1], Context.AllocationFeedback.bSaturatedNegative[1], PitchError);
	FAircraftPidGains YawGains   = MakeAntiWindupGains(Config.GetRatePidGains(2), Context.AllocationFeedback.bSaturatedPositive[2], Context.AllocationFeedback.bSaturatedNegative[2], YawError);

	if (Config.AngularDampingFeedForwardScale > UE_SMALL_NUMBER)
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
			Config.AngularDampingFeedForwardScale);
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


FVector FAircraftFlightControlSolver::ComputeDesiredHorizontalVelocity(const FAircraftFlightControlSolverContext& Context) const
{
	const FVector DesiredVelocity = Context.ManualCommand.DesiredVelocityCmPerSec;
	return FVector(DesiredVelocity.X, DesiredVelocity.Y, 0.0f);
}


FVector FAircraftFlightControlSolver::ComputeVelocityPidAcceleration(
	FAircraftFlightControlSolverContext& Context, const FVector& DesiredVelocityCmPerSec,
	const FVector& TrajectoryAccelerationFeedForwardCmPerSecSq, float DeltaSeconds)
{
	const FAircraftFlightControllerRuntimeConfig& Config = Context.Config;
	const FVector CurrentVelocity = Context.Runtime.EstimatedState.State.VelocityCmPerSec;
	const FVector DragFeedForward =
		FlightControlDynamics::ComputeLinearDampingFeedForward(
			DesiredVelocityCmPerSec, Context.PhysicsCache.LinearDampingPerSecond,
			Config.LinearDampingFeedForwardScale);
	const FVector TrajectoryFeedForward(
		TrajectoryAccelerationFeedForwardCmPerSecSq.X,
		TrajectoryAccelerationFeedForwardCmPerSecSq.Y,
		0.0f);
	const FVector TotalFeedForward = DragFeedForward + TrajectoryFeedForward;

	FVector DesiredAcceleration(
		PidStates.Velocity.X.UpdateFromMeasurement(
			DesiredVelocityCmPerSec.X, CurrentVelocity.X, DeltaSeconds,
			Config.GetVelocityPidGains(0), TotalFeedForward.X),
		PidStates.Velocity.Y.UpdateFromMeasurement(
			DesiredVelocityCmPerSec.Y, CurrentVelocity.Y, DeltaSeconds,
			Config.GetVelocityPidGains(1), TotalFeedForward.Y),
		0.0f);

	const float TiltLimitedAcceleration = Context.PhysicsCache.GravityMagnitudeCmPerSecSq
		* FMath::Tan(FMath::DegreesToRadians(Config.MaxTiltAngleDegrees));
	const float MaxHorizontalAcceleration = FMath::Min(
		Config.MaxHorizontalAccelerationCmPerSecSq, TiltLimitedAcceleration);
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


FVector FAircraftFlightControlSolver::ComputeDesiredHorizontalAcceleration(FAircraftFlightControlSolverContext& Context, float DeltaSeconds)
{
	const FAircraftFlightControllerRuntimeConfig& Config = Context.Config;
	const FVector CurrentPosition = Context.Runtime.EstimatedState.State.PositionCm;
	const float TiltLimitedAcceleration = Context.PhysicsCache.GravityMagnitudeCmPerSecSq
		* FMath::Tan(FMath::DegreesToRadians(Config.MaxTiltAngleDegrees));
	const float PhysicalHorizontalAcceleration = FMath::Min(
		Config.MaxHorizontalAccelerationCmPerSecSq, TiltLimitedAcceleration);
	const FlightControlDynamics::FDampingAwareHorizontalLimits DampingAwareLimits =
		FlightControlDynamics::ComputeDampingAwareHorizontalLimits(
			Config.MaxHorizontalSpeedCmPerSec, PhysicalHorizontalAcceleration,
			Context.PhysicsCache.LinearDampingPerSecond,
			Config.DampingAccelerationReserveFraction);
	const float ReachableHorizontalSpeed = DampingAwareLimits.MaxSpeedCmPerSec;

	if (!Context.ModeCapabilities.CanUsePositionControl && !Context.ModeCapabilities.CanUseVelocityControl)
	{
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
				PidStates.Position.X.UpdateFromMeasurement(AI.PositionSetpointCm.X, CurrentPosition.X, DeltaSeconds, Config.GetPositionPidGains(0), AI.VelocitySetpointCmPerSec.X),
				PidStates.Position.Y.UpdateFromMeasurement(AI.PositionSetpointCm.Y, CurrentPosition.Y, DeltaSeconds, Config.GetPositionPidGains(1), AI.VelocitySetpointCmPerSec.Y),
				0.0f);

			Context.Runtime.ControlOutput.bPositionTargetEnabled = true;
			Context.Runtime.ControlOutput.PositionTargetCm = FVector(AI.PositionSetpointCm.X, AI.PositionSetpointCm.Y, AI.AltitudeSetpointCm);
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

		Context.Runtime.ControlOutput.bVelocityTargetEnabled = true;
		Context.Runtime.ControlOutput.VelocityTargetCmPerSec.X = DesiredVelocity.X;
		Context.Runtime.ControlOutput.VelocityTargetCmPerSec.Y = DesiredVelocity.Y;

		// 速度 PID + 轨迹加速度前馈 + 维持目标速度所需的线性阻尼前馈。
		DesiredAcceleration = ComputeVelocityPidAcceleration(
			Context, DesiredVelocity, AI.AccelerationSetpointCmPerSecSq, DeltaSeconds);
		return DesiredAcceleration;
	}

	// ======================================================================
	// 手动摇杆路径
	// ======================================================================

	// 先计算摇杆对应的期望速度
	FVector DesiredVelocity = ComputeDesiredHorizontalVelocity(Context);

	// ---- 位置环（如果可用）----
	if (Context.ModeCapabilities.CanUsePositionControl)
	{
		const bool bManualHorizontalCommand = !FVector2D(
			Context.ManualCommand.DesiredVelocityCmPerSec.X,
			Context.ManualCommand.DesiredVelocityCmPerSec.Y).IsNearlyZero();

		if (!Context.Runtime.HoldTargets.bPositionHoldInitialized)
		{
			// 首次进入位置保持 → 锁定当前位置
			Context.Runtime.HoldTargets.HeldPositionCm = CurrentPosition;
			Context.Runtime.HoldTargets.bPositionHoldInitialized = true;
			PidStates.Position.X.Reset(); PidStates.Position.Y.Reset();
		}

		if (bManualHorizontalCommand)
		{
			// 手动输入时 → 重新锚定保持点，让位置 PID 不与手动指令打架
			Context.Runtime.HoldTargets.HeldPositionCm = CurrentPosition;
			Context.Runtime.HoldTargets.bHorizontalBrakeBeforeHold = true;
			PidStates.Position.X.Reset(); PidStates.Position.Y.Reset();
		}
		else if (Context.Runtime.HoldTargets.bHorizontalBrakeBeforeHold)
		{
			// 松杆是"制动到零速度"的过渡，而不是立即回到松杆点：
			// 制动期间锚点跟随机体，停稳后才锁定。
			Context.Runtime.HoldTargets.HeldPositionCm = CurrentPosition;
			PidStates.Position.X.Reset(); PidStates.Position.Y.Reset();
			const float HorizontalSpeed = FVector2D(
				Context.Runtime.EstimatedState.State.VelocityCmPerSec.X,
				Context.Runtime.EstimatedState.State.VelocityCmPerSec.Y).Size();
			if (HorizontalSpeed <= Config.HorizontalBrakeToHoldSpeedCmPerSec)
			{
				Context.Runtime.HoldTargets.bHorizontalBrakeBeforeHold = false;
			}
			DesiredVelocity = FVector::ZeroVector;
		}
		else
		{
			// 无手动输入 → 位置 PID 生成期望速度（导数对测量值，避免 kick）
			DesiredVelocity = FVector(
				PidStates.Position.X.UpdateFromMeasurement(Context.Runtime.HoldTargets.HeldPositionCm.X, CurrentPosition.X, DeltaSeconds, Config.GetPositionPidGains(0)),
				PidStates.Position.Y.UpdateFromMeasurement(Context.Runtime.HoldTargets.HeldPositionCm.Y, CurrentPosition.Y, DeltaSeconds, Config.GetPositionPidGains(1)),
				0.0);
		}

		Context.Runtime.ControlOutput.bPositionTargetEnabled = true;
		Context.Runtime.ControlOutput.PositionTargetCm = FVector(
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

	Context.Runtime.ControlOutput.bVelocityTargetEnabled = true;
	Context.Runtime.ControlOutput.VelocityTargetCmPerSec.X = DesiredVelocity.X;
	Context.Runtime.ControlOutput.VelocityTargetCmPerSec.Y = DesiredVelocity.Y;

	// ---- 速度 PID + 维持目标速度所需的线性阻尼前馈 ----
	return ComputeVelocityPidAcceleration(
		Context, DesiredVelocity, FVector::ZeroVector, DeltaSeconds);
}


float FAircraftFlightControlSolver::MapCenteredThrottleToCollective(const FAircraftFlightControlSolverContext& Context, float ThrottleInput) const
{
	const float HoverCollective = Context.Config.HoverCollectiveCommand;
	const float MinCollective = Context.Config.MinCollectiveCommand;
	const float MaxCollective = Context.Config.MaxCollectiveCommand;
	const float ClampedThrottle = FMath::Clamp(ThrottleInput, -1.0f, 1.0f);

	if (ClampedThrottle >= 0.0f)
	{
		return FMath::Lerp(HoverCollective, MaxCollective, ClampedThrottle);
	}
	return FMath::Lerp(HoverCollective, MinCollective, -ClampedThrottle);
}

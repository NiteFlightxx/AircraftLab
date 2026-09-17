
#include "Aircraft/FlightControlSolver.h"
#include "Aircraft/AircraftAttitudeReference.h"

#include "Aircraft/ControlAllocator.h"
#include "Math/RotationMatrix.h"

namespace
{
	FVector ComputeThrustVectorPriorityError(
		const FQuat& ActualControlWorldRotation,
		const FQuat& DesiredControlWorldRotation,
		const float YawWeight)
	{
		const FQuat Actual = ActualControlWorldRotation.GetNormalized();
		const FQuat Desired = DesiredControlWorldRotation.GetNormalized();
		const FVector ActualUp = Actual.RotateVector(FVector::UpVector);
		const FVector DesiredUp = Desired.RotateVector(FVector::UpVector);
		if (FVector::DotProduct(ActualUp, DesiredUp) < -1.0f + UE_KINDA_SMALL_NUMBER)
		{
			// The reduced attitude is singular for antiparallel thrust vectors.
			return AircraftAttitudeReference::GetShortestRotationVector(
				Actual, Desired);
		}

		const FQuat TiltDelta = FQuat::FindBetweenNormals(ActualUp, DesiredUp);
		FQuat ReducedDesired = (TiltDelta * Actual).GetNormalized();
		FQuat Residual = (ReducedDesired.Inverse() * Desired).GetNormalized();
		if (Residual.W < 0.0f)
		{
			Residual = FQuat(-Residual.X, -Residual.Y, -Residual.Z, -Residual.W);
		}
		const float ResidualYawRadians = 2.0f * FMath::Atan2(Residual.Z, Residual.W);
		const FVector TiltError =
			AircraftAttitudeReference::GetShortestRotationVector(
				Actual, ReducedDesired);
		return FVector(
			TiltError.X,
			TiltError.Y,
			FMath::Clamp(YawWeight, 0.0f, 1.0f) * ResidualYawRadians);
	}

	FAircraftAttitudeMotionConfig BuildFlightControllerAttitudeMotionConfig(
		const FAircraftFlightControllerRuntimeConfig& Config)
	{
		const float NaturalAngularFrequency = FMath::Max(
			Config.ReferenceModelNaturalAngularFrequencyRadPerSec, UE_SMALL_NUMBER);
		const float MaxTiltRateDegPerSec = FMath::Max(
			Config.MaxRollRateDegreesPerSec, Config.MaxPitchRateDegreesPerSec);
		FAircraftAttitudeMotionConfig Result;
		Result.MaxTiltAngleDegrees = Config.MaxTiltAngleDegrees;
		Result.NaturalFrequencyHz = NaturalAngularFrequency / UE_TWO_PI;
		Result.DampingRatio = 1.0f;
		Result.MaxAngularRateDegPerSec = FVector(
			Config.MaxRollRateDegreesPerSec,
			Config.MaxPitchRateDegreesPerSec,
			MaxTiltRateDegPerSec);
		Result.MaxAngularAccelerationDegPerSecSq = FVector(
			2.0f * NaturalAngularFrequency * Result.MaxAngularRateDegPerSec.X,
			2.0f * NaturalAngularFrequency * Result.MaxAngularRateDegPerSec.Y,
			2.0f * NaturalAngularFrequency * Result.MaxAngularRateDegPerSec.Z);
		Result.MaxAngularJerkDegPerSecCubed = FVector(
			2.0f * NaturalAngularFrequency * Result.MaxAngularAccelerationDegPerSecSq.X,
			2.0f * NaturalAngularFrequency * Result.MaxAngularAccelerationDegPerSecSq.Y,
			2.0f * NaturalAngularFrequency * Result.MaxAngularAccelerationDegPerSecSq.Z);
		Result.DynamicsFeedForwardScale = 0.0f;
		return Result;
	}
}

FVector FlightControlDynamics::ComputeLinearDampingFeedForward(
	const FVector& DesiredVelocityCmPerSec, const FVector& LinearDampingPerSecond, float Scale)
{
	const FVector EffectiveDamping = LinearDampingPerSecond.ComponentMax(FVector::ZeroVector)
		* FMath::Max(Scale, 0.0f);
	return FVector(
		DesiredVelocityCmPerSec.X * EffectiveDamping.X,
		DesiredVelocityCmPerSec.Y * EffectiveDamping.Y,
		0.0f);
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
	const FVector& DesiredBodyRatesDegPerSec, const FVector& AngularDampingPerSecond,
	const FVector& InertiaDiagonalKgM2, const FVector& PositiveTorqueAuthorityNm,
	const FVector& NegativeTorqueAuthorityNm, float Scale)
{
	const FVector EffectiveDamping = AngularDampingPerSecond.ComponentMax(FVector::ZeroVector)
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

void FAircraftFlightControlSolver::UpdateHoverThrustEstimate(
	const FAircraftFlightControllerRuntimeConfig& Config, float DeltaSeconds,
	float AccZWorldCmPerSecSq, float LastCollectiveThrustCommand, float GravityCmPerSecSq)
{
	if (!Config.HoverThrustEstimator.bEnabled)
	{
		return;
	}
	// 输入换算 cm→m：加速度与重力统一到 EKF 的 SI 约定
	HoverThrustEstimator.Update(DeltaSeconds, AccZWorldCmPerSecSq * 0.01f,
		LastCollectiveThrustCommand, GravityCmPerSecSq * 0.01f);
}

float FAircraftFlightControlSolver::GetEffectiveHoverCollectiveCommand(
	const FAircraftFlightControllerRuntimeConfig& Config) const
{
	return Config.HoverThrustEstimator.bEnabled && HoverThrustEstimator.IsInitialized()
		? HoverThrustEstimator.GetHoverThrust()
		: Config.HoverCollectiveCommand;
}

float FAircraftFlightControlSolver::ComputeVerticalControl(FAircraftFlightControlSolverContext& Context, float DeltaSeconds, float& OutDesiredVerticalVelocity)
{
	const FAircraftFlightControllerRuntimeConfig& Config = Context.Config;
	const float MinCollective = Config.MinCollectiveCommand;
	const float HoverCollective = GetEffectiveHoverCollectiveCommand(Config);
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
		Context.Runtime.HoldTargets.bVerticalBrakeBeforeHold = false;
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
		return MapCenteredThrottleToCollective(Context, NormalizedVerticalCommand, HoverCollective);
	}

	// 高度保持初始化（手动路径用；Autopilot 路径直接使用设定值，忽略此锁定值）
	if (!Context.Runtime.HoldTargets.bAltitudeHoldInitialized)
	{
		Context.Runtime.HoldTargets.HeldAltitudeCm = CurrentAltitude;
		Context.Runtime.HoldTargets.bAltitudeHoldInitialized = true;
		Context.Runtime.HoldTargets.bVerticalBrakeBeforeHold = false;
		PidStates.Altitude.Reset();
		PidStates.VerticalVelocity.Reset();
	}

	// ---- 统一轨迹参考 ----
	if (Context.bUseTrajectoryReference && Context.TrajectoryReference.bValid)
	{
		Context.Runtime.HoldTargets.bVerticalBrakeBeforeHold = false;
		const FAircraftTrajectoryReference& Reference = Context.TrajectoryReference;
		if (Reference.bPositionTrackingEnabled)
		{
			// 路径/Hold：高度外环叠加轨迹速度前馈。
			OutDesiredVerticalVelocity = PidStates.Altitude.UpdateFromMeasurement(
				Reference.PositionCm.Z, CurrentAltitude, DeltaSeconds,
				Config.GetAltitudePidGains(), Reference.VelocityCmPerSec.Z);
		}
		else
		{
			// Velocity 意图没有位置目标，不允许虚构一个逐帧位置点进入高度环。
			PidStates.Altitude.Reset();
			OutDesiredVerticalVelocity = Reference.VelocityCmPerSec.Z;
		}
		OutDesiredVerticalVelocity = FMath::Clamp(OutDesiredVerticalVelocity,
			-Config.MaxDescentRateCmPerSec, Config.MaxClimbRateCmPerSec);
		OutDesiredVerticalVelocity = SlewVerticalVelocitySetpoint(OutDesiredVerticalVelocity);
		// 预测参考拥有动力学补偿，飞控不再重复计算同一份阻尼前馈。
		LastVerticalDampingCollectiveFeedForward = 0.0f;
		const float CollectiveOffset = PidStates.VerticalVelocity.UpdateFromMeasurement(
			OutDesiredVerticalVelocity, CurrentVerticalVelocity, DeltaSeconds,
			Config.GetVerticalVelocityPidGains());
		const float Gravity = FMath::Max(Context.PhysicsCache.GravityMagnitudeCmPerSecSq, 1.0f);
		const float ReferenceAcceleration = Reference.ControlAccelerationCmPerSecSq.Z
			+ Reference.DynamicsFeedForwardAccelerationCmPerSecSq.Z;
		const float TrajectoryCollective = GetEffectiveHoverCollectiveCommand(Config)
			* FMath::Max(0.0f, (Gravity + ReferenceAcceleration) / Gravity);
		return FMath::Clamp(TrajectoryCollective + CollectiveOffset,
			MinCollective, MaxCollective);
	}

	// ---- 路径 B（手动）：高度保持 ----
	{
		// 油门杆在死区外 → 手动爬升/下降率，重新锚定高度
		const float RequestedVerticalVelocity = Context.ManualCommand.DesiredVelocityCmPerSec.Z;
		if (FMath::Abs(RequestedVerticalVelocity) > UE_SMALL_NUMBER)
		{
			OutDesiredVerticalVelocity = RequestedVerticalVelocity;
			// 手动爬升期间锚点跟随当前位置；松杆后先制动，再锁定最终高度。
			Context.Runtime.HoldTargets.HeldAltitudeCm = CurrentAltitude;
			Context.Runtime.HoldTargets.bVerticalBrakeBeforeHold = true;
			PidStates.Altitude.Reset();
		}
		else if (Context.Runtime.HoldTargets.bVerticalBrakeBeforeHold)
		{
			// 不在松杆瞬间冻结高度锚点。垂直速度设定值按最大加速度自然减到零，
			// 制动期间锚点持续跟随机体，避免越过松杆点后被高度环拉回。
			Context.Runtime.HoldTargets.HeldAltitudeCm = CurrentAltitude;
			PidStates.Altitude.Reset();
			OutDesiredVerticalVelocity = 0.0f;
			const float HoldSpeed = FMath::Max(Config.VerticalBrakeToHoldSpeedCmPerSec, 0.0f);
			const bool bSetpointStopped = !bVerticalVelocitySetpointInitialized
				|| FMath::Abs(LastDesiredVerticalVelocityCmPerSec) <= HoldSpeed;
			if (FMath::Abs(CurrentVerticalVelocity) <= HoldSpeed && bSetpointStopped)
			{
				Context.Runtime.HoldTargets.bVerticalBrakeBeforeHold = false;
			}
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
			OutDesiredVerticalVelocity, Context.PhysicsCache.LinearDampingPerSecond.Z,
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
	const float CurrentHeadingDegrees = AircraftAttitudeReference::GetPlanarHeadingDegrees(
		Context.PhysicsCache.BodyTransform.GetRotation().GetNormalized(), Config);

	return AircraftAttitudeReference::Build(
		DesiredHorizontalAcceleration,
		CurrentHeadingDegrees,
		GravityMagnitude,
		Config.MaxTiltAngleDegrees,
		Config).ControlWorldRotation.Rotator();
}


FAircraftYawSetpoint FAircraftFlightControlSolver::ComputeYawSetpoint(
	FAircraftFlightControlSolverContext& Context, const float DeltaSeconds)
{
	const FAircraftFlightControllerRuntimeConfig& Config = Context.Config;
	FAircraftYawSetpoint Result;
	// 航向保持初始化与下游航向误差必须使用同一份刚体四元数真值。
	const FQuat BodyWorldRotation = Context.PhysicsCache.BodyTransform.GetRotation().GetNormalized();
	const float CurrentYawDegrees = AircraftAttitudeReference::GetPlanarHeadingDegrees(
		BodyWorldRotation, Config);
	Result.TargetYawDegrees = CurrentYawDegrees;
	Result.MaxRateDegPerSec = Config.MaxYawRateDegreesPerSec;
	const bool bHasYawTorqueAuthority =
		Context.AllocationFeedback.Cache.PositiveTorqueAuthority[2]
			> AircraftAllocation::AuthorityEpsilon
		|| Context.AllocationFeedback.Cache.NegativeTorqueAuthority[2]
			> AircraftAllocation::AuthorityEpsilon;
	if (!bHasYawTorqueAuthority)
	{
		YawReferenceModel.Reset();
		Result.MaxRateDegPerSec = 0.0f;
		return Result;
	}

	// ---- 统一轨迹参考 ----
	if (Context.bUseTrajectoryReference && Context.TrajectoryReference.bValid)
	{
		YawReferenceModel.Reset();
		const FAircraftTrajectoryReference& Reference = Context.TrajectoryReference;
		if (!bTrajectoryPositionTrackingInitialized
			|| bLastTrajectoryPositionTrackingEnabled != Reference.bPositionTrackingEnabled)
		{
			// Hold/Path 与纯 Velocity 的外环语义不同；切换时不得继承上一语义的积分状态。
			PidStates.Position.X.Reset();
			PidStates.Position.Y.Reset();
			PidStates.Velocity.X.Reset();
			PidStates.Velocity.Y.Reset();
			bTrajectoryPositionTrackingInitialized = true;
			bLastTrajectoryPositionTrackingEnabled = Reference.bPositionTrackingEnabled;
		}
		const float IntentYawRateLimit = FMath::Max(
			Reference.YawRateLimitDegPerSec, 0.0f);
		Result.MaxRateDegPerSec = FMath::Min(Config.MaxYawRateDegreesPerSec, IntentYawRateLimit);
		if (Result.MaxRateDegPerSec <= UE_SMALL_NUMBER)
		{
			return Result;
		}
		Result.TargetYawDegrees = FRotator::NormalizeAxis(Reference.YawDegrees);
		Result.FeedForwardRateDegPerSec = FMath::Clamp(
			Reference.YawRateDegPerSec, -Result.MaxRateDegPerSec, Result.MaxRateDegPerSec);
		return Result;
	}

	// 手动路径与 Autopilot 使用同一个 jerk/加速度/速率受限参考动态。
	// 松杆时目标速率变为零，参考角继续按剩余角速度积分并自然制动；
	// 制动结束时的积分角度就是新的 Hold，而不是松杆瞬间冻结的旧角度。
	FAircraftYawReferenceLimits Limits;
	Limits.MaxRateDegPerSec = Result.MaxRateDegPerSec;
	Limits.MaxAccelerationDegPerSecSq = Config.MaxYawAccelerationDegPerSecSq;
	Limits.MaxJerkDegPerSecCubed = Config.MaxYawJerkDegPerSecCubed;
	Limits.ResponseTimeSeconds = 1.0f / FMath::Max(
		Config.ReferenceModelNaturalAngularFrequencyRadPerSec, UE_SMALL_NUMBER);
	const float MeasuredYawRateDegPerSec =
		AircraftAttitudeReference::GetPlanarHeadingRateDegreesPerSecond(
			BodyWorldRotation,
			Context.PhysicsCache.AngularVelocityWorldRadPerSec,
			Config.GetForwardAxisBody(), Config.GetRightAxisBody());
	if (!FAircraftYawReferenceDynamics::UpdateRateCommand(
		Context.ManualCommand.DesiredYawRateDegPerSec,
		CurrentYawDegrees, MeasuredYawRateDegPerSec,
		DeltaSeconds, Limits, YawReferenceModel))
	{
		YawReferenceModel.Reset();
		return Result;
	}
	Result.TargetYawDegrees = YawReferenceModel.YawDegrees;
	Result.FeedForwardRateDegPerSec = YawReferenceModel.RateDegPerSec;
	return Result;
}


FVector FAircraftFlightControlSolver::ComputeDesiredBodyRates(FAircraftFlightControlSolverContext& Context,
	const FRotator& DesiredAttitude, const FAircraftYawSetpoint& YawSetpoint, float DeltaSeconds)
{
	const FAircraftFlightControllerRuntimeConfig& Config = Context.Config;
	if (Context.Runtime.AttitudeMode == EAircraftAttitudeMode::Acro
		|| Context.Runtime.AttitudeMode == EAircraftAttitudeMode::Manual)
	{
		FAircraftAttitudeReferenceDynamics::Reset(AttitudeReferenceModel);
		return FVector(
			FMath::Clamp(Context.ManualCommand.DesiredBodyRatesDegPerSec.X,
				-Config.MaxRollRateDegreesPerSec, Config.MaxRollRateDegreesPerSec),
			FMath::Clamp(Context.ManualCommand.DesiredBodyRatesDegPerSec.Y,
				-Config.MaxPitchRateDegreesPerSec, Config.MaxPitchRateDegreesPerSec),
			FMath::Clamp(YawSetpoint.FeedForwardRateDegPerSec,
				-YawSetpoint.MaxRateDegPerSec, YawSetpoint.MaxRateDegPerSec));
	}

	const FQuat ActualBodyWorldRotation =
		Context.PhysicsCache.BodyTransform.GetRotation().GetNormalized();
	const FQuat ActualControlWorldRotation = Config.GetControlWorldRotation(
		ActualBodyWorldRotation);
	const FQuat RawControlWorldRotation = FRotator(
		DesiredAttitude.Pitch,
		YawSetpoint.TargetYawDegrees,
		DesiredAttitude.Roll).Quaternion();
	FQuat ShapedControlWorldRotation = RawControlWorldRotation;
	FVector ReferenceBodyRateRadPerSec = ActualBodyWorldRotation.UnrotateVector(
		FVector::UpVector * FMath::DegreesToRadians(
			YawSetpoint.FeedForwardRateDegPerSec));

	if (Config.bEnableAttitudeReferenceModel)
	{
		// YawDegrees/YawRate have already been shaped by the single authoritative
		// FAircraftYawReferenceDynamics instance. The SO(3) reference model owns only
		// Roll/Pitch tilt dynamics; otherwise two stateful yaw loops chase each other.
		const FRotator ActualControlAttitude = ActualControlWorldRotation.Rotator();
		const FQuat ActualTiltControlWorldRotation = FRotator(
			ActualControlAttitude.Pitch, 0.0f, ActualControlAttitude.Roll).Quaternion();
		const FQuat ActualTiltBodyWorldRotation = Config.GetBodyWorldRotation(
			ActualTiltControlWorldRotation);
		const FVector ActualAngularVelocityWorldRadPerSec =
			Context.PhysicsCache.AngularVelocityWorldRadPerSec;
		const float ActualHeadingRateDegPerSec =
			AircraftAttitudeReference::GetPlanarHeadingRateDegreesPerSecond(
				ActualBodyWorldRotation,
				Context.PhysicsCache.AngularVelocityWorldRadPerSec,
				Config.GetForwardAxisBody(), Config.GetRightAxisBody());
		const float ActualHeadingDegrees =
			AircraftAttitudeReference::GetPlanarHeadingDegrees(
				ActualBodyWorldRotation, Config);
		const FQuat ActualHeadingWorldRotation(
			FVector::UpVector, FMath::DegreesToRadians(ActualHeadingDegrees));
		const FVector ActualTiltAngularVelocityNeutralWorldRadPerSec =
			ActualHeadingWorldRotation.UnrotateVector(
				ActualAngularVelocityWorldRadPerSec
				- FVector::UpVector * FMath::DegreesToRadians(
					ActualHeadingRateDegPerSec));
		const FVector ActualTiltAngularVelocityBodyRadPerSec =
			ActualTiltBodyWorldRotation.UnrotateVector(
				ActualTiltAngularVelocityNeutralWorldRadPerSec);
		const FQuat RawTiltControlWorldRotation = FRotator(
			DesiredAttitude.Pitch, 0.0f, DesiredAttitude.Roll).Quaternion();
		FAircraftAttitudeMotionOutput ReferenceOutput;
		const FAircraftAttitudeMotionConfig MotionConfig =
			BuildFlightControllerAttitudeMotionConfig(Config);
		if (!FAircraftAttitudeReferenceDynamics::UpdateTiltTarget(
			RawTiltControlWorldRotation,
			DeltaSeconds,
			ActualTiltBodyWorldRotation,
			ActualTiltAngularVelocityBodyRadPerSec,
			Config,
			MotionConfig,
			AttitudeReferenceModel,
			ReferenceOutput))
		{
			FAircraftAttitudeReferenceDynamics::Reset(AttitudeReferenceModel);
			return FVector::ZeroVector;
		}
		const FRotator ShapedTiltAttitude =
			ReferenceOutput.ControlWorldRotation.Rotator();
		ShapedControlWorldRotation = FRotator(
			ShapedTiltAttitude.Pitch,
			YawSetpoint.TargetYawDegrees,
			ShapedTiltAttitude.Roll).Quaternion();
		const FVector ReferenceTiltControllerRateRadPerSec =
			Config.BodyAngularToController(
				ReferenceOutput.AngularVelocityBodyRadPerSec);
		const FVector ReferenceTiltBodyRateRadPerSec =
			Config.ControlToBodyVector(FVector(
				-ReferenceTiltControllerRateRadPerSec.X,
				-ReferenceTiltControllerRateRadPerSec.Y,
				0.0f));
		const FQuat ShapedBodyWorldRotation = Config.GetBodyWorldRotation(
			ShapedControlWorldRotation);
		const FVector AuthoritativeYawBodyRateRadPerSec =
			ShapedBodyWorldRotation.UnrotateVector(
				FVector::UpVector * FMath::DegreesToRadians(
					YawSetpoint.FeedForwardRateDegPerSec));
		ReferenceBodyRateRadPerSec = ReferenceTiltBodyRateRadPerSec
			+ AuthoritativeYawBodyRateRadPerSec;
	}
	else
	{
		FAircraftAttitudeReferenceDynamics::Reset(AttitudeReferenceModel);
	}

	FVector ReferenceControllerRateDegPerSec = FMath::RadiansToDegrees(
		Config.BodyAngularToController(ReferenceBodyRateRadPerSec));
	const float AttitudeRateFeedForwardLimit = FMath::Max(
		Config.ReferenceModelRateFeedForwardLimitDegPerSec, 0.0f);
	ReferenceControllerRateDegPerSec.X = FMath::Clamp(
		ReferenceControllerRateDegPerSec.X,
		-AttitudeRateFeedForwardLimit, AttitudeRateFeedForwardLimit);
	ReferenceControllerRateDegPerSec.Y = FMath::Clamp(
		ReferenceControllerRateDegPerSec.Y,
		-AttitudeRateFeedForwardLimit, AttitudeRateFeedForwardLimit);

	const float RollPitchGain = 0.5f
		* (FMath::Max(Config.AttitudeGains.X, 0.0f)
			+ FMath::Max(Config.AttitudeGains.Y, 0.0f));
	const float YawWeight = RollPitchGain > UE_SMALL_NUMBER
		? FMath::Clamp(Config.AttitudeGains.Z / RollPitchGain, 0.0f, 1.0f)
		: 1.0f;
	const FVector RotationErrorControl = ComputeThrustVectorPriorityError(
		ActualControlWorldRotation, ShapedControlWorldRotation, YawWeight);
	const float EffectiveYawGain = YawWeight > UE_SMALL_NUMBER
		? Config.AttitudeGains.Z / YawWeight
		: 0.0f;
	const FVector AttitudeCorrectionControllerRadPerSec(
		-RotationErrorControl.X * Config.AttitudeGains.X,
		-RotationErrorControl.Y * Config.AttitudeGains.Y,
		RotationErrorControl.Z * EffectiveYawGain);
	const FVector DesiredControllerRateDegPerSec = ReferenceControllerRateDegPerSec
		+ FMath::RadiansToDegrees(AttitudeCorrectionControllerRadPerSec);
	return FVector(
		FMath::Clamp(DesiredControllerRateDegPerSec.X,
			-Config.MaxRollRateDegreesPerSec, Config.MaxRollRateDegreesPerSec),
		FMath::Clamp(DesiredControllerRateDegPerSec.Y,
			-Config.MaxPitchRateDegreesPerSec, Config.MaxPitchRateDegreesPerSec),
		FMath::Clamp(DesiredControllerRateDegPerSec.Z,
			-YawSetpoint.MaxRateDegPerSec, YawSetpoint.MaxRateDegPerSec));
}


FVector FAircraftFlightControlSolver::ComputeBodyTorqueCommand(FAircraftFlightControlSolverContext& Context, const FVector& DesiredBodyRatesDegreesPerSec, float DeltaSeconds)
{
	const FAircraftFlightControllerRuntimeConfig& Config = Context.Config;
	const FVector CurrentBodyRates = Context.Runtime.EstimatedState.State.AngularVelocityBodyDegreesPerSec;

	// 上一帧分配器饱和方向只冻结本帧积分增量；已有积分仍参与输出，
	// 反向误差仍可卸载积分，避免临时 Ki=0 造成隐藏 windup 和解饱和跳变。
	auto AllowsIntegralAccumulation = [](bool bSaturatedPos, bool bSaturatedNeg,
		float RateError)
	{
		return !((bSaturatedPos && RateError > 0.0f)
			|| (bSaturatedNeg && RateError < 0.0f));
	};

	const float RollError  = DesiredBodyRatesDegreesPerSec.X - CurrentBodyRates.X;
	const float PitchError = DesiredBodyRatesDegreesPerSec.Y - CurrentBodyRates.Y;
	const float YawError   = DesiredBodyRatesDegreesPerSec.Z - CurrentBodyRates.Z;

	FAircraftPidGains RollGains = Config.GetRatePidGains(0);
	FAircraftPidGains PitchGains = Config.GetRatePidGains(1);
	FAircraftPidGains YawGains = Config.GetRatePidGains(2);
	const bool bIntegrateRoll = AllowsIntegralAccumulation(
		Context.AllocationFeedback.bSaturatedPositive[0],
		Context.AllocationFeedback.bSaturatedNegative[0], RollError);
	const bool bIntegratePitch = AllowsIntegralAccumulation(
		Context.AllocationFeedback.bSaturatedPositive[1],
		Context.AllocationFeedback.bSaturatedNegative[1], PitchError);
	const bool bIntegrateYaw = AllowsIntegralAccumulation(
		Context.AllocationFeedback.bSaturatedPositive[2],
		Context.AllocationFeedback.bSaturatedNegative[2], YawError);

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
	FVector Result(
		PidStates.Rate.Roll.UpdateFromMeasurement(DesiredBodyRatesDegreesPerSec.X,
			CurrentBodyRates.X, DeltaSeconds, RollGains,
			LastAngularDampingFeedForward.X, bIntegrateRoll),
		PidStates.Rate.Pitch.UpdateFromMeasurement(DesiredBodyRatesDegreesPerSec.Y,
			CurrentBodyRates.Y, DeltaSeconds, PitchGains,
			LastAngularDampingFeedForward.Y, bIntegratePitch),
		PidStates.Rate.Yaw.UpdateFromMeasurement(DesiredBodyRatesDegreesPerSec.Z,
			CurrentBodyRates.Z, DeltaSeconds, YawGains,
			LastAngularDampingFeedForward.Z, bIntegrateYaw));
	if (Context.AllocationFeedback.Cache.RowScale[1] <= AircraftAllocation::AuthorityEpsilon)
	{
		PidStates.Rate.Roll.Reset();
		Result.X = 0.0f;
	}
	if (Context.AllocationFeedback.Cache.RowScale[2] <= AircraftAllocation::AuthorityEpsilon)
	{
		PidStates.Rate.Pitch.Reset();
		Result.Y = 0.0f;
	}
	if (Context.AllocationFeedback.Cache.RowScale[3] <= AircraftAllocation::AuthorityEpsilon)
	{
		PidStates.Rate.Yaw.Reset();
		Result.Z = 0.0f;
	}
	return Result;
}


FVector FAircraftFlightControlSolver::ComputeDesiredHorizontalVelocity(const FAircraftFlightControlSolverContext& Context) const
{
	const FVector DesiredVelocity = Context.ManualCommand.DesiredVelocityCmPerSec;
	return FVector(DesiredVelocity.X, DesiredVelocity.Y, 0.0f);
}


FVector FAircraftFlightControlSolver::ComputeVelocityPidAcceleration(
	FAircraftFlightControlSolverContext& Context, const FVector& DesiredVelocityCmPerSec,
	const FVector& TrajectoryAccelerationFeedForwardCmPerSecSq, float DeltaSeconds,
	bool bIncludeLinearDampingFeedForward)
{
	const FAircraftFlightControllerRuntimeConfig& Config = Context.Config;
	const FVector CurrentVelocity = Context.Runtime.EstimatedState.State.VelocityCmPerSec;
	const FVector DragFeedForward = bIncludeLinearDampingFeedForward
		? FlightControlDynamics::ComputeLinearDampingFeedForward(
			DesiredVelocityCmPerSec, Context.PhysicsCache.LinearDampingPerSecond,
			Config.LinearDampingFeedForwardScale)
		: FVector::ZeroVector;
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
	const FVector RequestedAcceleration = DesiredAcceleration;
	const FVector VelocityFeedbackAcceleration = DesiredAcceleration - TotalFeedForward;

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

	// The physical tilt/vector limit is downstream of the two scalar PID clamps.
	// Feed the acceleration actually achievable by the aircraft back into each
	// integrator so a sustained velocity error cannot wind the horizontal loop up
	// behind that shared vector limit.
	PidStates.Velocity.X.ApplyTrackingAntiWindup(
		RequestedAcceleration.X, DesiredAcceleration.X, DeltaSeconds,
		Config.GetVelocityPidGains(0));
	PidStates.Velocity.Y.ApplyTrackingAntiWindup(
		RequestedAcceleration.Y, DesiredAcceleration.Y, DeltaSeconds,
		Config.GetVelocityPidGains(1));

	LastDesiredHorizontalVelocityCmPerSec = FVector(DesiredVelocityCmPerSec.X, DesiredVelocityCmPerSec.Y, 0.0f);
	LastVelocityDragFeedForwardCmPerSecSq = DragFeedForward;
	LastTrajectoryAccelerationFeedForwardCmPerSecSq = TrajectoryFeedForward;
	LastVelocityFeedbackAccelerationCmPerSecSq = VelocityFeedbackAcceleration;
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
			FMath::Max(Context.PhysicsCache.LinearDampingPerSecond.X,
				Context.PhysicsCache.LinearDampingPerSecond.Y),
			Config.DampingAccelerationReserveFraction);
	const float ReachableHorizontalSpeed = DampingAwareLimits.MaxSpeedCmPerSec;

	if (!Context.ModeCapabilities.CanUsePositionControl && !Context.ModeCapabilities.CanUseVelocityControl)
	{
		PidStates.Velocity.X.Reset(); PidStates.Velocity.Y.Reset();
		LastDesiredHorizontalVelocityCmPerSec = FVector::ZeroVector;
		LastVelocityDragFeedForwardCmPerSecSq = FVector::ZeroVector;
		LastTrajectoryAccelerationFeedForwardCmPerSecSq = FVector::ZeroVector;
		LastVelocityFeedbackAccelerationCmPerSecSq = FVector::ZeroVector;
		LastDesiredHorizontalAccelerationCmPerSecSq = FVector::ZeroVector;
		return FVector::ZeroVector;
	}

	// ======================================================================
	// 统一轨迹参考路径
	// ======================================================================
	if (Context.bUseTrajectoryReference && Context.TrajectoryReference.bValid)
	{
		const FAircraftTrajectoryReference& Reference = Context.TrajectoryReference;

		FVector DesiredVelocity = FVector::ZeroVector;
		FVector DesiredAcceleration = FVector::ZeroVector;

		if (Reference.bPositionTrackingEnabled
			&& Context.ModeCapabilities.CanUsePositionControl)
		{
			// 位置环：设定值 = PositionSetpointCm.XY，前馈 = VelocitySetpointCmPerSec.XY
			DesiredVelocity = FVector(
				PidStates.Position.X.UpdateFromMeasurement(Reference.PositionCm.X, CurrentPosition.X, DeltaSeconds, Config.GetPositionPidGains(0), Reference.VelocityCmPerSec.X),
				PidStates.Position.Y.UpdateFromMeasurement(Reference.PositionCm.Y, CurrentPosition.Y, DeltaSeconds, Config.GetPositionPidGains(1), Reference.VelocityCmPerSec.Y),
				0.0f);

			Context.Runtime.ControlOutput.bPositionTargetEnabled = true;
			Context.Runtime.ControlOutput.PositionTargetCm = Reference.PositionCm;
		}
		else
		{
			PidStates.Position.X.Reset();
			PidStates.Position.Y.Reset();
			DesiredVelocity = Reference.VelocityCmPerSec;
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

		// 预测参考已经分别给出轨迹运动学与动力学前馈，此处只合并一次。
		DesiredAcceleration = ComputeVelocityPidAcceleration(
			Context, DesiredVelocity,
			Reference.ControlAccelerationCmPerSecSq
				+ Reference.DynamicsFeedForwardAccelerationCmPerSecSq,
			DeltaSeconds, false);
		return DesiredAcceleration;
	}

	// ======================================================================
	// 手动摇杆路径
	// ======================================================================
	bTrajectoryPositionTrackingInitialized = false;

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


float FAircraftFlightControlSolver::MapCenteredThrottleToCollective(
	const FAircraftFlightControlSolverContext& Context, float ThrottleInput, float HoverCollective) const
{
	const float MinCollective = Context.Config.MinCollectiveCommand;
	const float MaxCollective = Context.Config.MaxCollectiveCommand;
	const float ClampedThrottle = FMath::Clamp(ThrottleInput, -1.0f, 1.0f);

	if (ClampedThrottle >= 0.0f)
	{
		return FMath::Lerp(HoverCollective, MaxCollective, ClampedThrottle);
	}
	return FMath::Lerp(HoverCollective, MinCollective, -ClampedThrottle);
}

#include "AircraftAutopilot/AircraftMotionPlan.h"
#include "ProfilingDebugging/CpuProfilerTrace.h"

namespace
{
	float ApplySoftLimit(float Requested, float Available, bool bHasRequestedLimits)
	{
		return bHasRequestedLimits ? FMath::Min(Requested, Available) : Available;
	}

	float WrapPlanTime(float Time, float Duration, bool bContinuous)
	{
		if (!bContinuous || Duration <= UE_SMALL_NUMBER)
		{
			return FMath::Clamp(Time, 0.0f, Duration);
		}
		const float Wrapped = FMath::Fmod(Time, Duration);
		return Wrapped < 0.0f ? Wrapped + Duration : Wrapped;
	}

	void ApplyCapabilityLimits(
		FAircraftRequestedMotionLimits& Limits,
		bool bHasRequestedLimits,
		const FAircraftDynamicCapabilitySnapshot& Capability)
	{
		Limits.CruiseSpeedCmPerSec = ApplySoftLimit(Limits.CruiseSpeedCmPerSec,
			Capability.MaxHorizontalSpeedCmPerSec, bHasRequestedLimits);
		Limits.MaxAccelerationCmPerSecSq = ApplySoftLimit(Limits.MaxAccelerationCmPerSecSq,
			Capability.MaxHorizontalAccelerationCmPerSecSq, bHasRequestedLimits);
		Limits.MaxDecelerationCmPerSecSq = ApplySoftLimit(Limits.MaxDecelerationCmPerSecSq,
			Capability.MaxHorizontalDecelerationCmPerSecSq, bHasRequestedLimits);
		Limits.MaxJerkCmPerSecCubed = ApplySoftLimit(Limits.MaxJerkCmPerSecCubed,
			Capability.MaxHorizontalJerkCmPerSecCubed, bHasRequestedLimits);
		Limits.MaxVerticalAccelerationCmPerSecSq = ApplySoftLimit(
			Limits.MaxVerticalAccelerationCmPerSecSq,
			Capability.MaxVerticalAccelerationCmPerSecSq, bHasRequestedLimits);
		Limits.MaxVerticalJerkCmPerSecCubed = ApplySoftLimit(
			Limits.MaxVerticalJerkCmPerSecCubed,
			Capability.MaxVerticalJerkCmPerSecCubed, bHasRequestedLimits);
		Limits.MaxClimbRateCmPerSec = ApplySoftLimit(Limits.MaxClimbRateCmPerSec,
			Capability.MaxClimbRateCmPerSec, bHasRequestedLimits);
		Limits.MaxDescentRateCmPerSec = ApplySoftLimit(Limits.MaxDescentRateCmPerSec,
			Capability.MaxDescentRateCmPerSec, bHasRequestedLimits);
		Limits.MaxYawRateDegPerSec = Capability.HasAngularControlAuthority(2)
			? ApplySoftLimit(Limits.MaxYawRateDegPerSec,
				FMath::RadiansToDegrees(Capability.MaxBodyRateRadPerSec.Z), bHasRequestedLimits)
			: 0.0f;
		Limits.MaxYawAccelerationDegPerSecSq = Capability.HasAngularControlAuthority(2)
			? ApplySoftLimit(Limits.MaxYawAccelerationDegPerSecSq,
				FMath::RadiansToDegrees(Capability.MaxBodyAngularAccelerationRadPerSecSq.Z),
				bHasRequestedLimits)
			: 0.0f;
		Limits.MaxYawJerkDegPerSecCubed = Capability.HasAngularControlAuthority(2)
			? ApplySoftLimit(Limits.MaxYawJerkDegPerSecCubed,
				FMath::RadiansToDegrees(Capability.MaxBodyAngularJerkRadPerSecCubed.Z),
				bHasRequestedLimits)
			: 0.0f;
	}

	float ResolveHorizontalThrustAuthority(
		const FAircraftDynamicCapabilitySnapshot& Capability)
	{
		float Result = Capability.MaxHorizontalAccelerationCmPerSecSq;
		if (Capability.bHasTiltLimit)
		{
			Result = FMath::Min(Result, Capability.GravityCmPerSecSq
				* FMath::Tan(Capability.MaxTiltRadians));
		}
		if (Capability.bHasTiltLimit && Capability.MassKg > UE_SMALL_NUMBER)
		{
			if (Capability.CollectiveAuthorityN <= UE_SMALL_NUMBER)
			{
				return 0.0f;
			}
			const float SpecificThrustCmPerSecSq =
				Capability.CollectiveAuthorityN * 100.0f / Capability.MassKg;
			const float HorizontalAuthority = FMath::Sqrt(FMath::Max(0.0f,
				FMath::Square(SpecificThrustCmPerSecSq)
				- FMath::Square(Capability.GravityCmPerSecSq)));
			Result = FMath::Min(Result, HorizontalAuthority);
		}
		return FMath::Max(Result, 0.0f);
	}

	float ComputeWorstCaseDragAcceleration(
		float SpeedCmPerSec, const FAircraftDynamicCapabilitySnapshot& Capability)
	{
		const float Damping = FMath::Max(
			Capability.LinearDampingPerSecond.X,
			Capability.LinearDampingPerSecond.Y);
		float Result = FMath::Max(Damping, 0.0f) * SpeedCmPerSec;
		if (!Capability.bHasExplicitAerodynamics
			|| Capability.MassKg <= UE_SMALL_NUMBER)
		{
			return Result;
		}
		const float BoundedSpeedCmPerSec = Capability.MaxRelativeAirspeedCmPerSec > 0.0f
			? FMath::Min(SpeedCmPerSec, Capability.MaxRelativeAirspeedCmPerSec)
			: SpeedCmPerSec;
		const float SpeedMps = BoundedSpeedCmPerSec * 0.01f;
		const float LinearDrag = FMath::Max3(
			Capability.LinearDragBodyNsPerM.X,
			Capability.LinearDragBodyNsPerM.Y,
			Capability.LinearDragBodyNsPerM.Z);
		const float AreaCoefficient = FMath::Max3(
			Capability.DragAreaCoefficientBodyM2.X,
			Capability.DragAreaCoefficientBodyM2.Y,
			Capability.DragAreaCoefficientBodyM2.Z);
		const float DragForceN = FMath::Max(LinearDrag, 0.0f) * SpeedMps
			+ 0.5f * Capability.AirDensityKgPerM3
				* FMath::Max(AreaCoefficient, 0.0f) * FMath::Square(SpeedMps);
		return Result + DragForceN * 100.0f / Capability.MassKg;
	}
}

void FAircraftMotionPlan::Reset()
{
	SourceIntent = {};
	SpatialPath.Reset();
	Samples.Reset();
	DurationSeconds = 0.0f;
	bContinuous = false;
	bValid = false;
}

float FAircraftMotionPlan::ResolveYaw(
	const FAircraftHeadingObjective& Heading, const FVector& PositionCm,
	const FVector& VelocityCmPerSec, float PreviousYawDegrees)
{
	switch (Heading.Mode)
	{
	case EAircraftHeadingMode::FixedYaw:
		return Heading.FixedYawDegrees;
	case EAircraftHeadingMode::FaceVelocity:
		return VelocityCmPerSec.SizeSquared2D() > 1.0
			? FMath::RadiansToDegrees(FMath::Atan2(VelocityCmPerSec.Y, VelocityCmPerSec.X))
			: PreviousYawDegrees;
	case EAircraftHeadingMode::FaceTarget:
	{
		const FVector Delta = Heading.TargetPositionCm - PositionCm;
		return Delta.SizeSquared2D() > 1.0
			? FMath::RadiansToDegrees(FMath::Atan2(Delta.Y, Delta.X))
			: PreviousYawDegrees;
	}
	default:
		return PreviousYawDegrees;
	}
}

bool FAircraftMotionPlan::BuildHoldPlan(
	const FAircraftMovementIntent& Intent, const FAircraftVehicleStateSnapshot& InitialState)
{
	FAircraftMotionPlanSample Sample;
	Sample.PositionCm = Intent.Hold.bCaptureCurrentPosition
		? InitialState.PositionCm : Intent.Hold.PositionCm;
	Sample.YawDegrees = ResolveYaw(Intent.Heading, Sample.PositionCm,
		FVector::ZeroVector, InitialState.ControlRotation.Rotator().Yaw);
	Samples.Add(Sample);
	bContinuous = true;
	return true;
}

bool FAircraftMotionPlan::BuildTimedPlan(const FAircraftMovementIntent& Intent)
{
	Samples.Reserve(Intent.TimedTrajectory.Samples.Num());
	float PreviousYawAccelerationDegPerSecSq = 0.0f;
	for (int32 Index = 0; Index < Intent.TimedTrajectory.Samples.Num(); ++Index)
	{
		const FAircraftTimedTrajectorySample& Input = Intent.TimedTrajectory.Samples[Index];
		const float HorizontalSpeedCmPerSec = FVector2D(
			Input.VelocityCmPerSec.X, Input.VelocityCmPerSec.Y).Size();
		const float VerticalRateLimitCmPerSec = Input.VelocityCmPerSec.Z >= 0.0
			? Intent.Limits.MaxClimbRateCmPerSec
			: Intent.Limits.MaxDescentRateCmPerSec;
		const FVector2D HorizontalAcceleration(
			Input.AccelerationCmPerSecSq.X, Input.AccelerationCmPerSecSq.Y);
		const FVector2D HorizontalVelocity(
			Input.VelocityCmPerSec.X, Input.VelocityCmPerSec.Y);
		const float HorizontalAccelerationLimitCmPerSecSq =
			FVector2D::DotProduct(HorizontalVelocity, HorizontalAcceleration) < 0.0f
				? Intent.Limits.MaxDecelerationCmPerSecSq
				: Intent.Limits.MaxAccelerationCmPerSecSq;
		if (HorizontalSpeedCmPerSec > Intent.Limits.CruiseSpeedCmPerSec + UE_KINDA_SMALL_NUMBER
			|| FMath::Abs(Input.VelocityCmPerSec.Z)
				> VerticalRateLimitCmPerSec + UE_KINDA_SMALL_NUMBER
			|| HorizontalAcceleration.Size()
				> HorizontalAccelerationLimitCmPerSecSq + UE_KINDA_SMALL_NUMBER
			|| FMath::Abs(Input.AccelerationCmPerSecSq.Z)
				> Intent.Limits.MaxVerticalAccelerationCmPerSecSq + UE_KINDA_SMALL_NUMBER
			|| FMath::Abs(Input.YawRateDegPerSec)
				> Intent.Limits.MaxYawRateDegPerSec + UE_KINDA_SMALL_NUMBER)
		{
			return false;
		}
		if (Index > 0)
		{
			const FAircraftTimedTrajectorySample& Previous =
				Intent.TimedTrajectory.Samples[Index - 1];
			const float Dt = Input.TimeSeconds - Previous.TimeSeconds;
			const FVector JerkCmPerSecCubed =
				(Input.AccelerationCmPerSecSq - Previous.AccelerationCmPerSecSq) / Dt;
			const float YawAccelerationDegPerSecSq =
				(Input.YawRateDegPerSec - Previous.YawRateDegPerSec) / Dt;
			const float YawJerkDegPerSecCubed = Index > 1
				? (YawAccelerationDegPerSecSq - PreviousYawAccelerationDegPerSecSq) / Dt
				: 0.0f;
			if (FVector2D(JerkCmPerSecCubed.X, JerkCmPerSecCubed.Y).Size()
					> Intent.Limits.MaxJerkCmPerSecCubed + UE_KINDA_SMALL_NUMBER
				|| FMath::Abs(JerkCmPerSecCubed.Z)
					> Intent.Limits.MaxVerticalJerkCmPerSecCubed + UE_KINDA_SMALL_NUMBER
				|| FMath::Abs(YawAccelerationDegPerSecSq)
					> Intent.Limits.MaxYawAccelerationDegPerSecSq + UE_KINDA_SMALL_NUMBER
				|| FMath::Abs(YawJerkDegPerSecCubed)
					> Intent.Limits.MaxYawJerkDegPerSecCubed + UE_KINDA_SMALL_NUMBER)
			{
				return false;
			}
			PreviousYawAccelerationDegPerSecSq = YawAccelerationDegPerSecSq;
		}
		FAircraftMotionPlanSample& Sample = Samples.AddDefaulted_GetRef();
		Sample.TimeSeconds = Input.TimeSeconds;
		Sample.PositionCm = Input.PositionCm;
		Sample.VelocityCmPerSec = Input.VelocityCmPerSec;
		Sample.AccelerationCmPerSecSq = Input.AccelerationCmPerSecSq;
		Sample.YawDegrees = Input.YawDegrees;
		Sample.YawRateDegPerSec = Input.YawRateDegPerSec;
		if (Samples.Num() > 1)
		{
			Sample.DistanceCm = Samples[Samples.Num() - 2].DistanceCm
				+ FVector::Distance(Samples[Samples.Num() - 2].PositionCm, Sample.PositionCm);
		}
	}
	DurationSeconds = Samples.Last().TimeSeconds;
	return Samples.Num() >= 2 && DurationSeconds > 0.0f;
}

bool FAircraftMotionPlan::BuildSpatialPlan(
	const FAircraftMovementIntent& Intent,
	const FAircraftAutopilotRuntimeConfig& Config,
	const FAircraftVehicleStateSnapshot& InitialState,
	const FAircraftDynamicCapabilitySnapshot& Capability)
{
	FAircraftRouteIntent Route = Intent.Route;
	if (Intent.Type == EAircraftMovementIntentType::Orbit)
	{
		constexpr int32 OrbitPointCount = 64;
		Route.bClosed = true;
		Route.PointsCm.Reserve(OrbitPointCount);
		const float Sign = Intent.Orbit.AngularRateDegPerSec >= 0.0f ? 1.0f : -1.0f;
		const FVector Offset = InitialState.PositionCm - Intent.Orbit.CenterCm;
		const float StartAngle = Offset.SizeSquared2D() > 1.0
			? FMath::Atan2(Offset.Y, Offset.X) : 0.0f;
		for (int32 Index = 0; Index < OrbitPointCount; ++Index)
		{
			const float Angle = StartAngle + Sign * UE_TWO_PI
				* static_cast<float>(Index) / static_cast<float>(OrbitPointCount);
			Route.PointsCm.Add(Intent.Orbit.CenterCm + FVector(
				FMath::Cos(Angle) * Intent.Orbit.RadiusCm,
				FMath::Sin(Angle) * Intent.Orbit.RadiusCm,
				Offset.Z));
		}
	}
	if (!SpatialPath.Build(Route, Config.Path))
	{
		return false;
	}

	bContinuous = Route.bClosed;
	const float Length = SpatialPath.GetLengthCm();
	const int32 Count = FMath::Max(2, FMath::CeilToInt(Length / Config.Timing.SampleSpacingCm) + 1);
	Samples.SetNum(Count);
	TArray<float> SpeedLimits;
	SpeedLimits.SetNum(Count);
	const float RequestedAcceleration = Intent.Limits.MaxAccelerationCmPerSecSq;
	const float RequestedDeceleration = Intent.Limits.MaxDecelerationCmPerSecSq;
	float CruiseSpeed = Intent.Limits.CruiseSpeedCmPerSec;
	const float PhysicalHorizontalAcceleration = ResolveHorizontalThrustAuthority(Capability);
	const float UsableThrustAcceleration = PhysicalHorizontalAcceleration
		* (1.0f - Config.Timing.ThrustReserveFraction);
	if (Capability.bValid && PhysicalHorizontalAcceleration > UE_SMALL_NUMBER
		&& ComputeWorstCaseDragAcceleration(CruiseSpeed, Capability)
			> UsableThrustAcceleration)
	{
		float LowSpeed = 0.0f;
		float HighSpeed = CruiseSpeed;
		for (int32 Iteration = 0; Iteration < 24; ++Iteration)
		{
			const float CandidateSpeed = 0.5f * (LowSpeed + HighSpeed);
			if (ComputeWorstCaseDragAcceleration(CandidateSpeed, Capability)
				<= UsableThrustAcceleration)
			{
				LowSpeed = CandidateSpeed;
			}
			else
			{
				HighSpeed = CandidateSpeed;
			}
		}
		CruiseSpeed = LowSpeed;
	}
	const float AccelerationLimit = FMath::Min(RequestedAcceleration, UsableThrustAcceleration);
	const float CurvatureAccelerationLimit = AccelerationLimit
		* (1.0f - Config.Timing.CurvatureAccelerationReserveFraction);
	const float DecelerationLimit = FMath::Min(RequestedDeceleration,
		Capability.MaxHorizontalDecelerationCmPerSecSq)
		* (1.0f - Config.Timing.BrakingReserveFraction);

	for (int32 Index = 0; Index < Count; ++Index)
	{
		const float Distance = Length * static_cast<float>(Index) / static_cast<float>(Count - 1);
		FAircraftSpatialPathState PathState;
		SpatialPath.Evaluate(Distance, PathState);
		FAircraftMotionPlanSample& Sample = Samples[Index];
		Sample.DistanceCm = Distance;
		Sample.PositionCm = PathState.PositionCm;
		const float Curvature = static_cast<float>(PathState.CurvaturePerCm.Size());
		float Limit = CruiseSpeed;
		if (Curvature > UE_SMALL_NUMBER && CurvatureAccelerationLimit > 0.0f)
		{
			float LowSpeed = 0.0f;
			float HighSpeed = Limit;
			for (int32 Iteration = 0; Iteration < 20; ++Iteration)
			{
				const float CandidateSpeed = 0.5f * (LowSpeed + HighSpeed);
				const FVector NormalAcceleration = PathState.CurvaturePerCm
					* FMath::Square(CandidateSpeed);
				const float DragAcceleration = Capability.bValid
					? ComputeWorstCaseDragAcceleration(CandidateSpeed, Capability) : 0.0f;
				const FVector RequiredNetAcceleration = NormalAcceleration
					+ PathState.Tangent * DragAcceleration;
				const FVector RequiredSpecificThrust = RequiredNetAcceleration
					+ FVector(0.0, 0.0, Capability.GravityCmPerSecSq);
				const float MaximumSpecificThrust = Capability.CollectiveAuthorityN > UE_SMALL_NUMBER
					&& Capability.MassKg > UE_SMALL_NUMBER
					? Capability.CollectiveAuthorityN * 100.0f / Capability.MassKg
						* (1.0f - Config.Timing.ThrustReserveFraction)
					: TNumericLimits<float>::Max();
				const float TiltLimitedHorizontalAcceleration = Capability.bHasTiltLimit
					? FMath::Max(RequiredSpecificThrust.Z, 0.0f)
						* FMath::Tan(Capability.MaxTiltRadians)
					: TNumericLimits<float>::Max();
				const bool bFeasible = NormalAcceleration.SizeSquared()
					<= FMath::Square(CurvatureAccelerationLimit)
					&& FVector2D(RequiredNetAcceleration.X, RequiredNetAcceleration.Y).SizeSquared()
						<= FMath::Square(UsableThrustAcceleration)
					&& (Capability.MaxVerticalAccelerationCmPerSecSq <= 0.0f
						|| FMath::Abs(NormalAcceleration.Z)
							<= Capability.MaxVerticalAccelerationCmPerSecSq)
					&& RequiredSpecificThrust.SizeSquared()
						<= FMath::Square(MaximumSpecificThrust)
					&& FVector2D(RequiredSpecificThrust.X, RequiredSpecificThrust.Y).SizeSquared()
						<= FMath::Square(TiltLimitedHorizontalAcceleration);
				if (bFeasible)
				{
					LowSpeed = CandidateSpeed;
				}
				else
				{
					HighSpeed = CandidateSpeed;
				}
			}
			Limit = LowSpeed;
		}
		const float AbsVerticalTangent = FMath::Abs(static_cast<float>(PathState.Tangent.Z));
		if (AbsVerticalTangent > UE_SMALL_NUMBER)
		{
			const float VerticalRate = PathState.Tangent.Z >= 0.0
				? Intent.Limits.MaxClimbRateCmPerSec : Intent.Limits.MaxDescentRateCmPerSec;
			Limit = FMath::Min(Limit, VerticalRate / AbsVerticalTangent);
		}
		SpeedLimits[Index] = FMath::Max(0.0f, Limit);
		Sample.VelocityCmPerSec = PathState.Tangent;
		Sample.AccelerationCmPerSecSq = PathState.CurvaturePerCm;
	}

	// 航向是轨迹可达性的一部分。高速通过急转弯但同时把偏航速率截断，
	// 会让机体朝向、阻力方向和控制轴全部脱离空间路径。
	if (Intent.Limits.MaxYawRateDegPerSec > UE_SMALL_NUMBER)
	{
		TArray<float> PathYawDegrees;
		PathYawDegrees.SetNum(Count);
		float PreviousYaw = InitialState.ControlRotation.Rotator().Yaw;
		for (int32 Index = 0; Index < Count; ++Index)
		{
			PathYawDegrees[Index] = ResolveYaw(Intent.Heading, Samples[Index].PositionCm,
				Samples[Index].VelocityCmPerSec, PreviousYaw);
			PreviousYaw = PathYawDegrees[Index];
		}
		for (int32 Index = 1; Index < Count; ++Index)
		{
			const float DistanceDelta = Samples[Index].DistanceCm
				- Samples[Index - 1].DistanceCm;
			const float YawDelta = FMath::Abs(FMath::FindDeltaAngleDegrees(
				PathYawDegrees[Index - 1], PathYawDegrees[Index]));
			if (DistanceDelta > UE_SMALL_NUMBER && YawDelta > UE_SMALL_NUMBER)
			{
				const float YawLimitedSpeed = Intent.Limits.MaxYawRateDegPerSec
					* DistanceDelta / YawDelta;
				SpeedLimits[Index - 1] = FMath::Min(SpeedLimits[Index - 1], YawLimitedSpeed);
				SpeedLimits[Index] = FMath::Min(SpeedLimits[Index], YawLimitedSpeed);
			}
		}
	}

	const float InitialAlongTrackSpeed = FMath::Max(0.0f,
		static_cast<float>(FVector::DotProduct(InitialState.VelocityCmPerSec, Samples[0].VelocityCmPerSec)));
	if (!bContinuous)
	{
		SpeedLimits[0] = FMath::Min(SpeedLimits[0], InitialAlongTrackSpeed);
		SpeedLimits.Last() = FMath::Min(SpeedLimits.Last(),
			Intent.Completion.ArrivalMode == EAircraftArrivalMode::Stop
				? 0.0f : Intent.Completion.TerminalSpeedCmPerSec);
	}

	for (int32 Iteration = 0; Iteration < Config.Timing.MaxIterations; ++Iteration)
	{
		const TArray<float> PreviousSpeedLimits = SpeedLimits;
		for (int32 Index = 1; Index < Count; ++Index)
		{
			const float Ds = Samples[Index].DistanceCm - Samples[Index - 1].DistanceCm;
			const float DragAcceleration = Capability.bValid
				? ComputeWorstCaseDragAcceleration(SpeedLimits[Index - 1], Capability) : 0.0f;
			const float NormalAcceleration = Samples[Index - 1].AccelerationCmPerSecSq.Size()
				* FMath::Square(SpeedLimits[Index - 1]);
			const float RemainingTangentialAuthority = FMath::Sqrt(FMath::Max(0.0f,
				FMath::Square(UsableThrustAcceleration)
				- FMath::Square(NormalAcceleration)));
			const float AvailableAcceleration = FMath::Min(AccelerationLimit,
				FMath::Max(RemainingTangentialAuthority - DragAcceleration, 0.0f));
			SpeedLimits[Index] = FMath::Min(SpeedLimits[Index], FMath::Sqrt(FMath::Max(0.0f,
				FMath::Square(SpeedLimits[Index - 1]) + 2.0f * AvailableAcceleration * Ds)));
		}
		for (int32 Index = Count - 2; Index >= 0; --Index)
		{
			const float Ds = Samples[Index + 1].DistanceCm - Samples[Index].DistanceCm;
			SpeedLimits[Index] = FMath::Min(SpeedLimits[Index], FMath::Sqrt(FMath::Max(0.0f,
				FMath::Square(SpeedLimits[Index + 1]) + 2.0f * DecelerationLimit * Ds)));
		}
		if (bContinuous)
		{
			const float SeamSpeed = FMath::Min(SpeedLimits[0], SpeedLimits.Last());
			SpeedLimits[0] = SeamSpeed;
			SpeedLimits.Last() = SeamSpeed;
		}
		float MaximumChange = 0.0f;
		for (int32 Index = 0; Index < Count; ++Index)
		{
			MaximumChange = FMath::Max(MaximumChange,
				FMath::Abs(SpeedLimits[Index] - PreviousSpeedLimits[Index]));
		}
		if (MaximumChange <= Config.Timing.SpeedConvergenceToleranceCmPerSec)
		{
			break;
		}
	}

	// 在速度包络上继续施加双向 jerk 可达性。前向约束加速度增长，反向约束
	// 加速度下降（进入制动）；只允许降低既有速度，因此不会破坏曲率、推力或终端上限。
	auto SegmentDeltaTime = [this, &SpeedLimits, AccelerationLimit, DecelerationLimit](int32 Index)
	{
		const float Ds = Samples[Index + 1].DistanceCm - Samples[Index].DistanceCm;
		const float SumSpeed = SpeedLimits[Index] + SpeedLimits[Index + 1];
		return SumSpeed > UE_SMALL_NUMBER
			? 2.0f * Ds / SumSpeed
			: FMath::Sqrt(2.0f * Ds
				/ FMath::Max(FMath::Max(AccelerationLimit, DecelerationLimit), 1.0f));
	};
	auto SegmentAcceleration = [this, &SpeedLimits](int32 Index)
	{
		const float Ds = Samples[Index + 1].DistanceCm - Samples[Index].DistanceCm;
		return Ds > UE_SMALL_NUMBER
			? (FMath::Square(SpeedLimits[Index + 1])
				- FMath::Square(SpeedLimits[Index])) / (2.0f * Ds)
			: 0.0f;
	};
	auto TangentialJerkLimit = [this, &Intent](int32 Index)
	{
		const FVector Tangent = Samples[Index].VelocityCmPerSec;
		float Limit = TNumericLimits<float>::Max();
		const float HorizontalTangent = FVector2D(Tangent.X, Tangent.Y).Size();
		if (HorizontalTangent > UE_SMALL_NUMBER)
		{
			Limit = FMath::Min(Limit,
				Intent.Limits.MaxJerkCmPerSecCubed / HorizontalTangent);
		}
		if (FMath::Abs(Tangent.Z) > UE_SMALL_NUMBER)
		{
			Limit = FMath::Min(Limit,
				Intent.Limits.MaxVerticalJerkCmPerSecCubed / FMath::Abs(Tangent.Z));
		}
		return Limit == TNumericLimits<float>::Max() ? 0.0f : FMath::Max(Limit, 0.0f);
	};
	const int32 SegmentCount = Count - 1;
	const int32 MaximumJerkPropagationSweeps = Config.Timing.MaxIterations
		* FMath::Max(SegmentCount, 1);
	for (int32 Iteration = 0; Iteration < MaximumJerkPropagationSweeps; ++Iteration)
	{
		float MaximumSpeedChange = 0.0f;
		float PreviousAcceleration = bContinuous
			? SegmentAcceleration(SegmentCount - 1)
			: FMath::Clamp(static_cast<float>(FVector::DotProduct(
				InitialState.AccelerationCmPerSecSq, Samples[0].VelocityCmPerSec)),
				-DecelerationLimit, AccelerationLimit);
		for (int32 Index = 0; Index < SegmentCount; ++Index)
		{
			const float Ds = Samples[Index + 1].DistanceCm - Samples[Index].DistanceCm;
			const float Dt = SegmentDeltaTime(Index);
			const float CurrentAcceleration = SegmentAcceleration(Index);
			const float MaximumAcceleration = PreviousAcceleration
				+ TangentialJerkLimit(Index) * Dt;
			if (CurrentAcceleration > MaximumAcceleration)
			{
				const float PreviousSpeed = SpeedLimits[Index + 1];
				SpeedLimits[Index + 1] = FMath::Min(SpeedLimits[Index + 1],
					FMath::Sqrt(FMath::Max(0.0f,
						FMath::Square(SpeedLimits[Index]) + 2.0f * MaximumAcceleration * Ds)));
				MaximumSpeedChange = FMath::Max(MaximumSpeedChange,
					PreviousSpeed - SpeedLimits[Index + 1]);
			}
			PreviousAcceleration = SegmentAcceleration(Index);
		}

		float NextAcceleration = bContinuous ? SegmentAcceleration(0) : 0.0f;
		for (int32 Index = SegmentCount - 1; Index >= 0; --Index)
		{
			const float Ds = Samples[Index + 1].DistanceCm - Samples[Index].DistanceCm;
			const int32 JerkIntervalIndex = FMath::Min(Index + 1, SegmentCount - 1);
			const float Dt = SegmentDeltaTime(JerkIntervalIndex);
			const float CurrentAcceleration = SegmentAcceleration(Index);
			const float MaximumAccelerationBeforeNext = NextAcceleration
				+ TangentialJerkLimit(JerkIntervalIndex) * Dt;
			if (CurrentAcceleration > MaximumAccelerationBeforeNext)
			{
				const float PreviousSpeed = SpeedLimits[Index + 1];
				SpeedLimits[Index + 1] = FMath::Min(SpeedLimits[Index + 1],
					FMath::Sqrt(FMath::Max(0.0f,
						FMath::Square(SpeedLimits[Index])
							+ 2.0f * MaximumAccelerationBeforeNext * Ds)));
				MaximumSpeedChange = FMath::Max(MaximumSpeedChange,
					PreviousSpeed - SpeedLimits[Index + 1]);
			}
			NextAcceleration = SegmentAcceleration(Index);
		}
		if (bContinuous)
		{
			const float SeamSpeed = FMath::Min(SpeedLimits[0], SpeedLimits.Last());
			SpeedLimits[0] = SeamSpeed;
			SpeedLimits.Last() = SeamSpeed;
		}
		if (MaximumSpeedChange <= Config.Timing.SpeedConvergenceToleranceCmPerSec)
		{
			break;
		}
	}

	float PreviousYaw = InitialState.ControlRotation.Rotator().Yaw;
	for (int32 Index = 0; Index < Count; ++Index)
	{
		FAircraftMotionPlanSample& Sample = Samples[Index];
		const FVector Tangent = Sample.VelocityCmPerSec;
		const FVector Curvature = Sample.AccelerationCmPerSecSq;
		Sample.VelocityCmPerSec = Tangent * SpeedLimits[Index];
		if (Index > 0)
		{
			const float Ds = Sample.DistanceCm - Samples[Index - 1].DistanceCm;
			const float SumSpeed = SpeedLimits[Index - 1] + SpeedLimits[Index];
			const float Dt = SumSpeed > UE_SMALL_NUMBER
				? 2.0f * Ds / SumSpeed
				: FMath::Sqrt(2.0f * Ds / FMath::Max(AccelerationLimit, 1.0f));
			Sample.TimeSeconds = Samples[Index - 1].TimeSeconds + Dt;
		}
		const float TangentialAcceleration = Index > 0 && Sample.TimeSeconds > Samples[Index - 1].TimeSeconds
			? (SpeedLimits[Index] - SpeedLimits[Index - 1])
				/ (Sample.TimeSeconds - Samples[Index - 1].TimeSeconds) : 0.0f;
		Sample.AccelerationCmPerSecSq = Tangent * TangentialAcceleration
			+ Curvature * FMath::Square(SpeedLimits[Index]);
		Sample.YawDegrees = ResolveYaw(Intent.Heading, Sample.PositionCm,
			Sample.VelocityCmPerSec, PreviousYaw);
		if (Index > 0)
		{
			const float Dt = Sample.TimeSeconds - Samples[Index - 1].TimeSeconds;
			Sample.YawRateDegPerSec = Dt > UE_SMALL_NUMBER
				? FMath::FindDeltaAngleDegrees(PreviousYaw, Sample.YawDegrees) / Dt : 0.0f;
			Sample.YawRateDegPerSec = FMath::Clamp(Sample.YawRateDegPerSec,
				-Intent.Limits.MaxYawRateDegPerSec, Intent.Limits.MaxYawRateDegPerSec);
		}
		PreviousYaw = Sample.YawDegrees;
	}
	DurationSeconds = Samples.Last().TimeSeconds;
	return DurationSeconds > 0.0f;
}

bool FAircraftMotionPlan::Build(
	const FAircraftMovementIntent& Intent,
	const FAircraftAutopilotRuntimeConfig& Config,
	const FAircraftVehicleStateSnapshot& InitialState,
	const FAircraftDynamicCapabilitySnapshot& Capability)
{
	TRACE_CPUPROFILER_EVENT_SCOPE(Aircraft_Trajectory_Build);
	Reset();
	if (!Intent.IsValid() || !Config.IsValid() || !Capability.bValid)
	{
		return false;
	}
	SourceIntent = Intent;
	ApplyCapabilityLimits(SourceIntent.Limits, SourceIntent.bHasRequestedMotionLimits, Capability);
	switch (SourceIntent.Type)
	{
	case EAircraftMovementIntentType::Hold:
		bValid = BuildHoldPlan(SourceIntent, InitialState);
		break;
	case EAircraftMovementIntentType::Route:
	case EAircraftMovementIntentType::Orbit:
		bValid = BuildSpatialPlan(SourceIntent, Config, InitialState, Capability);
		break;
	case EAircraftMovementIntentType::TimedTrajectory:
		bValid = BuildTimedPlan(SourceIntent);
		break;
	case EAircraftMovementIntentType::Velocity:
		bContinuous = true;
		bValid = true;
		break;
	default:
		break;
	}
	return bValid;
}

bool FAircraftMotionPlan::Evaluate(float TimeSeconds, FAircraftMotionPlanSample& OutSample) const
{
	OutSample = {};
	if (Samples.IsEmpty())
	{
		return false;
	}
	if (Samples.Num() == 1)
	{
		OutSample = Samples[0];
		return true;
	}
	const float Time = WrapPlanTime(TimeSeconds, DurationSeconds, bContinuous);
	const int32 Upper = Algo::LowerBoundBy(Samples, Time,
		[](const FAircraftMotionPlanSample& Sample) { return Sample.TimeSeconds; });
	if (Upper <= 0)
	{
		OutSample = Samples[0];
		return true;
	}
	if (Upper >= Samples.Num())
	{
		OutSample = Samples.Last();
		return true;
	}
	const FAircraftMotionPlanSample& A = Samples[Upper - 1];
	const FAircraftMotionPlanSample& B = Samples[Upper];
	const float Alpha = FMath::GetRangePct(A.TimeSeconds, B.TimeSeconds, Time);
	OutSample.TimeSeconds = Time;
	OutSample.DistanceCm = FMath::Lerp(A.DistanceCm, B.DistanceCm, Alpha);
	OutSample.PositionCm = FMath::Lerp(A.PositionCm, B.PositionCm, Alpha);
	OutSample.VelocityCmPerSec = FMath::Lerp(A.VelocityCmPerSec, B.VelocityCmPerSec, Alpha);
	OutSample.AccelerationCmPerSecSq = FMath::Lerp(A.AccelerationCmPerSecSq, B.AccelerationCmPerSecSq, Alpha);
	OutSample.YawDegrees = A.YawDegrees + FMath::FindDeltaAngleDegrees(A.YawDegrees, B.YawDegrees) * Alpha;
	OutSample.YawRateDegPerSec = FMath::Lerp(A.YawRateDegPerSec, B.YawRateDegPerSec, Alpha);
	return true;
}

float FAircraftMotionPlan::TimeAtDistance(float DistanceCm) const
{
	if (Samples.Num() < 2)
	{
		return 0.0f;
	}
	const float Distance = bContinuous && GetLengthCm() > UE_SMALL_NUMBER
		? FMath::Fmod(FMath::Max(0.0f, DistanceCm), GetLengthCm())
		: FMath::Clamp(DistanceCm, 0.0f, Samples.Last().DistanceCm);
	const int32 Upper = Algo::LowerBoundBy(Samples, Distance,
		[](const FAircraftMotionPlanSample& Sample) { return Sample.DistanceCm; });
	if (Upper <= 0) return Samples[0].TimeSeconds;
	if (Upper >= Samples.Num()) return Samples.Last().TimeSeconds;
	const FAircraftMotionPlanSample& A = Samples[Upper - 1];
	const FAircraftMotionPlanSample& B = Samples[Upper];
	return FMath::Lerp(A.TimeSeconds, B.TimeSeconds,
		FMath::GetRangePct(A.DistanceCm, B.DistanceCm, Distance));
}

bool FAircraftMotionPlan::Project(
	const FVector& PositionCm, float InitialDistanceCm, bool bGlobalSearch,
	FAircraftMotionPlanSample& OutSample) const
{
	if (!SpatialPath.IsValid())
	{
		return Evaluate(0.0f, OutSample);
	}
	FAircraftSpatialPathState Projection;
	if (!SpatialPath.Project(PositionCm, InitialDistanceCm, bGlobalSearch, Projection))
	{
		return false;
	}
	if (!Evaluate(TimeAtDistance(Projection.DistanceCm), OutSample))
	{
		return false;
	}
	OutSample.DistanceCm = Projection.DistanceCm;
	OutSample.PositionCm = Projection.PositionCm;
	OutSample.VelocityCmPerSec = Projection.Tangent
		* static_cast<float>(OutSample.VelocityCmPerSec.Size());
	return true;
}

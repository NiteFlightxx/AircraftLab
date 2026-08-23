#include "AircraftAutopilot/AircraftMotionPlan.h"

namespace
{
	float PositiveMinimum(float A, float B)
	{
		if (A <= 0.0f) return B;
		if (B <= 0.0f) return A;
		return FMath::Min(A, B);
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
	for (const FAircraftTimedTrajectorySample& Input : Intent.TimedTrajectory.Samples)
	{
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
	const float PhysicalHorizontalAcceleration = Capability.bValid
		? Capability.MaxHorizontalAccelerationCmPerSecSq : RequestedAcceleration;
	const float AccelerationLimit = PositiveMinimum(RequestedAcceleration,
		PhysicalHorizontalAcceleration) * (1.0f - Config.Timing.ThrustReserveFraction);
	const float CurvatureAccelerationLimit = AccelerationLimit
		* (1.0f - Config.Timing.TorqueReserveFraction);
	const float DecelerationLimit = PositiveMinimum(RequestedDeceleration,
		PhysicalHorizontalAcceleration) * (1.0f - Config.Timing.BrakingReserveFraction);
	const float CruiseSpeed = PositiveMinimum(Intent.Limits.CruiseSpeedCmPerSec,
		Capability.bValid ? Capability.MaxHorizontalSpeedCmPerSec : 0.0f);

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
			Limit = FMath::Min(Limit, FMath::Sqrt(CurvatureAccelerationLimit / Curvature));
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
			SpeedLimits[Index] = FMath::Min(SpeedLimits[Index], FMath::Sqrt(FMath::Max(0.0f,
				FMath::Square(SpeedLimits[Index - 1]) + 2.0f * AccelerationLimit * Ds)));
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
		if (MaximumChange <= Config.Timing.FeasibilityTolerance)
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
	Reset();
	if (!Intent.IsValid() || !Config.IsValid())
	{
		return false;
	}
	SourceIntent = Intent;
	switch (Intent.Type)
	{
	case EAircraftMovementIntentType::Hold:
		bValid = BuildHoldPlan(Intent, InitialState);
		break;
	case EAircraftMovementIntentType::Route:
	case EAircraftMovementIntentType::Orbit:
		bValid = BuildSpatialPlan(Intent, Config, InitialState, Capability);
		break;
	case EAircraftMovementIntentType::TimedTrajectory:
		bValid = BuildTimedPlan(Intent);
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
	const FVector& PositionCm, float InitialDistanceCm, FAircraftMotionPlanSample& OutSample) const
{
	if (!SpatialPath.IsValid())
	{
		return Evaluate(0.0f, OutSample);
	}
	FAircraftSpatialPathState Projection;
	if (!SpatialPath.Project(PositionCm, InitialDistanceCm, Projection))
	{
		return false;
	}
	return Evaluate(TimeAtDistance(Projection.DistanceCm), OutSample);
}

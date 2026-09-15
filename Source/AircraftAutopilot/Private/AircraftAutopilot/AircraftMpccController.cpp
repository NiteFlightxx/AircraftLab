#include "AircraftAutopilot/AircraftMpccController.h"

#include "AircraftAutopilotDynamics.h"
#include "HAL/PlatformTime.h"
#include "ProfilingDebugging/CpuProfilerTrace.h"

namespace
{
	/** jerk 种子滤波截止频率：抑制物理差分加速度噪声，保留真实机动动态。 */
	constexpr float JerkSeedFilterCutoffHz = 5.0f;
	constexpr double OptimizationRelativeCostTolerance = 1.0e-4;
	constexpr double OptimizationCorrectionToleranceCmPerSecSq = 0.01;
	constexpr uint64 FnvOffsetBasis = 1469598103934665603ull;
	constexpr uint64 FnvPrime = 1099511628211ull;

	void HashBytes(uint64& Hash, const void* Data, const SIZE_T Size)
	{
		const uint8* Bytes = static_cast<const uint8*>(Data);
		for (SIZE_T Index = 0; Index < Size; ++Index)
		{
			Hash = (Hash ^ Bytes[Index]) * FnvPrime;
		}
	}

	template <typename T>
	void HashScalar(uint64& Hash, const T& Value)
	{
		HashBytes(Hash, &Value, sizeof(T));
	}

	void HashVector(uint64& Hash, const FVector& Value)
	{
		HashScalar(Hash, Value.X);
		HashScalar(Hash, Value.Y);
		HashScalar(Hash, Value.Z);
	}

	void HashMotionLimits(uint64& Hash, const FAircraftRequestedMotionLimits& Limits)
	{
		HashScalar(Hash, Limits.CruiseSpeedCmPerSec);
		HashScalar(Hash, Limits.MaxAccelerationCmPerSecSq);
		HashScalar(Hash, Limits.MaxDecelerationCmPerSecSq);
		HashScalar(Hash, Limits.MaxJerkCmPerSecCubed);
		HashScalar(Hash, Limits.MaxClimbRateCmPerSec);
		HashScalar(Hash, Limits.MaxDescentRateCmPerSec);
		HashScalar(Hash, Limits.MaxVerticalAccelerationCmPerSecSq);
		HashScalar(Hash, Limits.MaxVerticalJerkCmPerSecCubed);
		HashScalar(Hash, Limits.MaxYawRateDegPerSec);
		HashScalar(Hash, Limits.MaxYawAccelerationDegPerSecSq);
		HashScalar(Hash, Limits.MaxYawJerkDegPerSecCubed);
	}

	void HashPlanTimingSemantics(uint64& Hash, const FAircraftMovementIntent& Intent)
	{
		if (Intent.Type != EAircraftMovementIntentType::TimedTrajectory)
		{
			HashScalar(Hash, Intent.Heading.Mode);
			HashScalar(Hash, Intent.Heading.FixedYawDegrees);
			HashScalar(Hash, Intent.Heading.YawRateDegPerSec);
			HashVector(Hash, Intent.Heading.TargetPositionCm);
		}
		HashScalar(Hash, Intent.Completion.ArrivalMode);
		if (Intent.Completion.ArrivalMode == EAircraftArrivalMode::PassThrough)
		{
			HashScalar(Hash, Intent.Completion.TerminalHorizontalSpeedCmPerSec);
			HashScalar(Hash, Intent.Completion.TerminalVerticalSpeedCmPerSec);
		}
	}

	uint64 ComputePlanGeometryHash(const FAircraftMovementIntent& Intent)
	{
		uint64 Hash = FnvOffsetBasis;
		HashScalar(Hash, Intent.Type);
		HashScalar(Hash, Intent.bHasRequestedMotionLimits);
		if (Intent.bHasRequestedMotionLimits)
		{
			HashMotionLimits(Hash, Intent.Limits);
		}
		HashPlanTimingSemantics(Hash, Intent);
		switch (Intent.Type)
		{
		case EAircraftMovementIntentType::Hold:
			HashVector(Hash, Intent.Hold.PositionCm);
			HashScalar(Hash, Intent.Hold.bCaptureCurrentPosition);
			break;
		case EAircraftMovementIntentType::Velocity:
			HashVector(Hash, Intent.Velocity.VelocityCmPerSec);
			HashScalar(Hash, Intent.Velocity.Frame);
			break;
		case EAircraftMovementIntentType::Route:
			HashScalar(Hash, Intent.Route.bClosed);
			HashScalar(Hash, Intent.Route.PointsCm.Num());
			for (const FVector& Point : Intent.Route.PointsCm)
			{
				HashVector(Hash, Point);
			}
			HashScalar(Hash, Intent.Route.Corridor.Num());
			for (const FAircraftSafeCorridorSegment& Segment : Intent.Route.Corridor)
			{
				HashVector(Hash, Segment.AxisStartCm);
				HashVector(Hash, Segment.AxisEndCm);
				HashScalar(Hash, Segment.RadiusCm);
				HashScalar(Hash, Segment.StartDistanceCm);
				HashScalar(Hash, Segment.EndDistanceCm);
			}
			break;
		case EAircraftMovementIntentType::Orbit:
			HashVector(Hash, Intent.Orbit.CenterCm);
			HashScalar(Hash, Intent.Orbit.RadiusCm);
			HashScalar(Hash, Intent.Orbit.AngularRateDegPerSec);
			break;
		case EAircraftMovementIntentType::TimedTrajectory:
			HashScalar(Hash, Intent.TimedTrajectory.Samples.Num());
			for (const FAircraftTimedTrajectorySample& Sample : Intent.TimedTrajectory.Samples)
			{
				HashScalar(Hash, Sample.TimeSeconds);
				HashVector(Hash, Sample.PositionCm);
				HashVector(Hash, Sample.VelocityCmPerSec);
				HashVector(Hash, Sample.AccelerationCmPerSecSq);
				HashScalar(Hash, Sample.YawDegrees);
				HashScalar(Hash, Sample.YawRateDegPerSec);
			}
			break;
		default:
			break;
		}
		return Hash;
	}

	uint64 ComputeIntentMetadataHash(const FAircraftMovementIntent& Intent)
	{
		uint64 Hash = FnvOffsetBasis;
		HashScalar(Hash, Intent.Completion.HorizontalToleranceCm);
		HashScalar(Hash, Intent.Completion.VerticalToleranceCm);
		HashScalar(Hash, Intent.Completion.HorizontalSpeedToleranceCmPerSec);
		HashScalar(Hash, Intent.Completion.VerticalSpeedToleranceCmPerSec);
		HashScalar(Hash, Intent.Completion.YawToleranceDegrees);
		HashScalar(Hash, Intent.Completion.StableTimeSeconds);
		HashScalar(Hash, Intent.TimeoutSeconds);
		return Hash;
	}

	uint64 ComputePlanConfigHash(const FAircraftAutopilotRuntimeConfig& Config)
	{
		uint64 Hash = FnvOffsetBasis;
#define UE_AIRCRAFT_HASH_CONFIG(Value) HashScalar(Hash, Value)
		UE_AIRCRAFT_HASH_CONFIG(Config.Path.ResampleSpacingCm);
		UE_AIRCRAFT_HASH_CONFIG(Config.Path.MinimumSegmentLengthCm);
		UE_AIRCRAFT_HASH_CONFIG(Config.Path.CorridorSafetyMarginCm);
		UE_AIRCRAFT_HASH_CONFIG(Config.Path.ProjectionBacktrackToleranceCm);
		UE_AIRCRAFT_HASH_CONFIG(Config.Path.ProjectionSearchDistanceCm);
		UE_AIRCRAFT_HASH_CONFIG(Config.Path.CenterlineWeight);
		UE_AIRCRAFT_HASH_CONFIG(Config.Path.CurvatureWeight);
		UE_AIRCRAFT_HASH_CONFIG(Config.Path.SnapWeight);
		UE_AIRCRAFT_HASH_CONFIG(Config.Path.MaxIterations);
		UE_AIRCRAFT_HASH_CONFIG(Config.Path.ConvergenceToleranceCm);
		UE_AIRCRAFT_HASH_CONFIG(Config.Timing.SampleSpacingCm);
		UE_AIRCRAFT_HASH_CONFIG(Config.Timing.ThrustReserveFraction);
		UE_AIRCRAFT_HASH_CONFIG(Config.Timing.CurvatureAccelerationReserveFraction);
		UE_AIRCRAFT_HASH_CONFIG(Config.Timing.BrakingReserveFraction);
		UE_AIRCRAFT_HASH_CONFIG(Config.Timing.MaxIterations);
		UE_AIRCRAFT_HASH_CONFIG(Config.Timing.SpeedConvergenceToleranceCmPerSec);
#undef UE_AIRCRAFT_HASH_CONFIG
		return Hash;
	}

	// 与 TrajectoryRuntime::HardLimit 同语义：0 = 未请求（用 Available）而非零预算。
	// 当前上游 ApplyCapabilityLimits 已保证 Requested > 0，此处为跨实现的防御性一致。
	float ResolveHardLimit(float Requested, float Available)
	{
		if (Requested <= 0.0f) return Available;
		if (Available <= 0.0f) return Requested;
		return FMath::Min(Requested, Available);
	}

	bool PlanningCapabilityEquals(
		const FAircraftDynamicCapabilitySnapshot& A,
		const FAircraftDynamicCapabilitySnapshot& B)
	{
		// 容差按各字段物理量级的 ~0.1% 典型值选取：能力快照每子步重算，
		// 用 FMath::IsNearlyEqual 默认容差（1e-8）相当于逐位比较，
		// 任意 ULP 抖动都会触发整计划重建 + 进度调速器复位。
		return A.bValid == B.bValid
			&& FMath::IsNearlyEqual(A.MassKg, B.MassKg, 1.0e-3f)
			&& A.InertiaKgM2.Equals(B.InertiaKgM2, 1.0e-3)
			&& A.CenterOfMassBodyCm.Equals(B.CenterOfMassBodyCm, 0.01)
			&& FMath::IsNearlyEqual(A.GravityCmPerSecSq, B.GravityCmPerSecSq, 1.0f)
			&& FMath::IsNearlyEqual(A.MaxHorizontalSpeedCmPerSec, B.MaxHorizontalSpeedCmPerSec, 1.0f)
			&& FMath::IsNearlyEqual(A.MaxHorizontalAccelerationCmPerSecSq, B.MaxHorizontalAccelerationCmPerSecSq, 5.0f)
			&& FMath::IsNearlyEqual(A.MaxHorizontalDecelerationCmPerSecSq, B.MaxHorizontalDecelerationCmPerSecSq, 5.0f)
			&& FMath::IsNearlyEqual(A.MaxHorizontalJerkCmPerSecCubed, B.MaxHorizontalJerkCmPerSecCubed, 20.0f)
			&& FMath::IsNearlyEqual(A.MaxVerticalAccelerationCmPerSecSq, B.MaxVerticalAccelerationCmPerSecSq, 5.0f)
			&& FMath::IsNearlyEqual(A.MaxVerticalJerkCmPerSecCubed, B.MaxVerticalJerkCmPerSecCubed, 20.0f)
			&& FMath::IsNearlyEqual(A.MaxClimbRateCmPerSec, B.MaxClimbRateCmPerSec, 1.0f)
			&& FMath::IsNearlyEqual(A.MaxDescentRateCmPerSec, B.MaxDescentRateCmPerSec, 1.0f)
			&& FMath::IsNearlyEqual(A.MaxTiltRadians, B.MaxTiltRadians, 1.0e-3f)
			&& A.bHasTiltLimit == B.bHasTiltLimit
			&& A.MaxBodyRateRadPerSec.Equals(B.MaxBodyRateRadPerSec, 1.0e-4)
			&& A.MaxBodyAngularAccelerationRadPerSecSq.Equals(B.MaxBodyAngularAccelerationRadPerSecSq, 1.0e-4)
			&& A.MaxBodyAngularJerkRadPerSecCubed.Equals(B.MaxBodyAngularJerkRadPerSecCubed, 1.0e-4)
			&& A.bCanControlRoll == B.bCanControlRoll
			&& A.bCanControlPitch == B.bCanControlPitch
			&& A.bCanControlYaw == B.bCanControlYaw
			&& FMath::IsNearlyEqual(A.CollectiveAuthorityN, B.CollectiveAuthorityN, 0.05f)
			&& A.PositiveTorqueAuthorityNm.Equals(B.PositiveTorqueAuthorityNm, 1.0e-3)
			&& A.NegativeTorqueAuthorityNm.Equals(B.NegativeTorqueAuthorityNm, 1.0e-3)
			&& A.LinearDampingPerSecond.Equals(B.LinearDampingPerSecond, 1.0e-4)
			&& A.AngularDampingPerSecond.Equals(B.AngularDampingPerSecond, 1.0e-4)
			&& A.bHasExplicitAerodynamics == B.bHasExplicitAerodynamics
			&& FMath::IsNearlyEqual(A.AirDensityKgPerM3, B.AirDensityKgPerM3, 1.0e-3f)
			&& A.AircraftToBodyRotation.Equals(B.AircraftToBodyRotation, 1.0e-4)
			&& A.LinearDragAircraftNsPerM.Equals(B.LinearDragAircraftNsPerM, 1.0e-4)
			&& A.DragAreaCoefficientAircraftM2.Equals(B.DragAreaCoefficientAircraftM2, 1.0e-4)
			&& FMath::IsNearlyEqual(
				A.MaxRelativeAirspeedCmPerSec, B.MaxRelativeAirspeedCmPerSec, 1.0f)
			&& FMath::IsNearlyEqual(A.ThrustRiseResponseTimeSeconds, B.ThrustRiseResponseTimeSeconds, 1.0e-3f)
			&& FMath::IsNearlyEqual(A.ThrustFallResponseTimeSeconds, B.ThrustFallResponseTimeSeconds, 1.0e-3f);
	}
}

void FAircraftMpccController::Reset()
{
	Plan.Reset();
	RequestedIntent = {};
	PlanningCapability = {};
	RuntimeConfig = {};
	Diagnostics = {};
	LastReference = {};
	LastVelocityProfileAccelerationCmPerSecSq = FVector::ZeroVector;
	CandidateVelocityProfileAccelerationCmPerSecSq = FVector::ZeroVector;
	ControlCorrectionHorizon.Reset();
	ReferenceScratch.Reset();
	CommandAccelerationScratch.Reset();
	PositionScratch.Reset();
	VelocityScratch.Reset();
	RealizedAccelerationScratch.Reset();
	ResponseAlphaScratch.Reset();
	GradientScratch.Reset();
	PreviousCorrectionScratch.Reset();
	FilteredAccelerationCmPerSecSq = FVector::ZeroVector;
	LastFilterUpdateTimeSeconds = 0.0;
	bFilterInitialized = false;
	IntentRevision = 0;
	ActiveIntentId = 0;
	PlanRevision = 0;
	PlanGeometryHash = 0;
	IntentMetadataHash = 0;
	PlanConfigHash = 0;
	EstimatedPlanTimeSeconds = 0.0f;
	EstimatedDistanceCm = 0.0f;
	PathReferenceScale = 1.0f;
	PlanStartTimeSeconds = 0.0;
	LastPlanSolveTimeSeconds = 0.0;
	NextSolveTimeSeconds = -DBL_MAX;
	LastProjectionStatePositionCm = FVector::ZeroVector;
	LastProjectionStateTimeSeconds = 0.0;
	bProjectionStateInitialized = false;
}

bool FAircraftMpccController::SetIntent(
	const FAircraftMovementIntent& Intent, int64 InIntentId, uint64 InIntentRevision,
	const FAircraftAutopilotRuntimeConfig& Config,
	const FAircraftVehicleStateSnapshot& State,
	const FAircraftDynamicCapabilitySnapshot& Capability)
{
	// The metadata fast path must enforce the same input contract as a full build.
	// Otherwise an invalid heading/completion/config update could bypass Plan::Build
	// and poison the live controller while leaving the old plan marked valid.
	if (!Intent.IsValid() || !Config.IsValid() || !Capability.bValid)
	{
		Diagnostics.bPlanValid = false;
		return false;
	}
	const uint64 NewGeometryHash = ComputePlanGeometryHash(Intent);
	const uint64 NewMetadataHash = ComputeIntentMetadataHash(Intent);
	const uint64 NewPlanConfigHash = ComputePlanConfigHash(Config);
	const bool bSamePlanDefinition = Plan.IsValid()
		&& ActiveIntentId == InIntentId
		&& PlanGeometryHash == NewGeometryHash
		&& PlanConfigHash == NewPlanConfigHash;
	if (bSamePlanDefinition)
	{
		const bool bMetadataChanged = IntentMetadataHash != NewMetadataHash;
		RuntimeConfig = Config;
		RequestedIntent = Intent;
		IntentRevision = InIntentRevision;
		ActiveIntentId = InIntentId;
		IntentMetadataHash = NewMetadataHash;
		if (bMetadataChanged)
		{
			Plan.UpdateMetadata(Intent);
		}
		Diagnostics.ActiveIntentId = InIntentId;
		Diagnostics.IntentRevision = InIntentRevision;
		Diagnostics.PlanRevision = PlanRevision;
		Diagnostics.bPlanValid = true;
		NextSolveTimeSeconds = -DBL_MAX;
		return RefreshPlanForCapability(State, Capability);
	}
	const bool bPreserveReferenceState = Plan.IsValid()
		&& ActiveIntentId == InIntentId
		&& Plan.GetIntent().Type == Intent.Type;
	const bool bPreserveSpatialProgress = bPreserveReferenceState
		&& (Intent.Type == EAircraftMovementIntentType::Route
			|| Intent.Type == EAircraftMovementIntentType::Orbit);
	const bool bPreserveVelocityProfile = bPreserveReferenceState
		&& Intent.Type == EAircraftMovementIntentType::Velocity;
	const float PreviousEstimatedDistanceCm = EstimatedDistanceCm;
	const float PreviousPathReferenceScale = PathReferenceScale;
	RuntimeConfig = Config;
	RequestedIntent = Intent;
	PlanningCapability = Capability;
	IntentRevision = InIntentRevision;
	ActiveIntentId = InIntentId;
	PlanGeometryHash = NewGeometryHash;
	IntentMetadataHash = NewMetadataHash;
	PlanConfigHash = NewPlanConfigHash;
	Diagnostics = {};
	Diagnostics.ActiveIntentId = InIntentId;
	Diagnostics.IntentRevision = InIntentRevision;
	ControlCorrectionHorizon.Reset();
	EstimatedPlanTimeSeconds = 0.0f;
	EstimatedDistanceCm = bPreserveSpatialProgress ? PreviousEstimatedDistanceCm : 0.0f;
	PathReferenceScale = bPreserveSpatialProgress ? PreviousPathReferenceScale : 1.0f;
	PlanStartTimeSeconds = State.TimeSeconds;
	LastPlanSolveTimeSeconds = State.TimeSeconds;
	if (!bPreserveReferenceState)
	{
		LastReference = {};
	}
	if (!bPreserveVelocityProfile)
	{
		LastVelocityProfileAccelerationCmPerSecSq = FVector::ZeroVector;
		CandidateVelocityProfileAccelerationCmPerSecSq = FVector::ZeroVector;
	}
	// 新 revision 必须立即求解；同一 handle、同一意图类型保留连续参考状态。
	NextSolveTimeSeconds = -DBL_MAX;
	const bool bBuilt = Plan.Build(Intent, Config, State, Capability);
	if (bBuilt)
	{
		++PlanRevision;
		if (Intent.Type == EAircraftMovementIntentType::Route
			|| Intent.Type == EAircraftMovementIntentType::Orbit)
		{
			FAircraftMotionPlanSample InitialProjection;
			if (Plan.Project(State.PositionCm,
				bPreserveSpatialProgress ? PreviousEstimatedDistanceCm : 0.0f,
				!bPreserveSpatialProgress, InitialProjection))
			{
				EstimatedDistanceCm = InitialProjection.DistanceCm;
				EstimatedPlanTimeSeconds = Plan.TimeAtDistance(EstimatedDistanceCm);
			}
		}
		LastProjectionStatePositionCm = State.PositionCm;
		LastProjectionStateTimeSeconds = State.TimeSeconds;
		bProjectionStateInitialized = true;
	}
	Diagnostics.PlanRevision = PlanRevision;
	Diagnostics.bPlanValid = bBuilt;
	return bBuilt;
}

bool FAircraftMpccController::RefreshPlanForCapability(
	const FAircraftVehicleStateSnapshot& State,
	const FAircraftDynamicCapabilitySnapshot& Capability)
{
	if (PlanningCapabilityEquals(PlanningCapability, Capability))
	{
		return true;
	}

	FAircraftMotionPlan RebuiltPlan;
	if (!RebuiltPlan.Build(RequestedIntent, RuntimeConfig, State, Capability))
	{
		Diagnostics.bPlanValid = false;
		return false;
	}
	Plan = MoveTemp(RebuiltPlan);
	PlanningCapability = Capability;
	ControlCorrectionHorizon.Reset();
	PathReferenceScale = 1.0f;
	NextSolveTimeSeconds = -DBL_MAX;
	++PlanRevision;
	if (RequestedIntent.Type == EAircraftMovementIntentType::Route
		|| RequestedIntent.Type == EAircraftMovementIntentType::Orbit)
	{
		FAircraftMotionPlanSample Projection;
		if (Plan.Project(State.PositionCm, EstimatedDistanceCm, false, Projection))
		{
			EstimatedDistanceCm = Projection.DistanceCm;
			EstimatedPlanTimeSeconds = Plan.TimeAtDistance(EstimatedDistanceCm);
		}
	}
	LastProjectionStatePositionCm = State.PositionCm;
	LastProjectionStateTimeSeconds = State.TimeSeconds;
	bProjectionStateInitialized = true;
	Diagnostics.PlanRevision = PlanRevision;
	Diagnostics.bPlanValid = true;
	return true;
}

FVector FAircraftMpccController::ProjectAcceleration(
	const FVector& Acceleration,
	const FAircraftRequestedMotionLimits& Limits,
	const FAircraftDynamicCapabilitySnapshot& Capability,
	const FVector& VelocityCmPerSec)
{
	FVector Result = Acceleration;
	const bool bHorizontalBraking = FVector2D::DotProduct(
		FVector2D(Result.X, Result.Y), FVector2D(VelocityCmPerSec.X, VelocityCmPerSec.Y)) < 0.0f;
	const float CapabilityHorizontalLimit = bHorizontalBraking
		? Capability.MaxHorizontalDecelerationCmPerSecSq
		: Capability.MaxHorizontalAccelerationCmPerSecSq;
	const float HorizontalLimit = ResolveHardLimit(
		bHorizontalBraking ? Limits.MaxDecelerationCmPerSecSq : Limits.MaxAccelerationCmPerSecSq,
		CapabilityHorizontalLimit);
	const FVector2D Horizontal(Result.X, Result.Y);
	if (HorizontalLimit > 0.0f && Horizontal.SizeSquared() > FMath::Square(HorizontalLimit))
	{
		const FVector2D Clamped = Horizontal.GetSafeNormal() * HorizontalLimit;
		Result.X = Clamped.X;
		Result.Y = Clamped.Y;
	}
	const float VerticalLimit = ResolveHardLimit(
		Limits.MaxVerticalAccelerationCmPerSecSq, Capability.MaxVerticalAccelerationCmPerSecSq);
	if (VerticalLimit > 0.0f)
	{
		Result.Z = FMath::Clamp(Result.Z, -VerticalLimit, VerticalLimit);
	}
	return Result;
}

FVector FAircraftMpccController::ProjectControlAcceleration(
	const FVector& Acceleration,
	const FAircraftDynamicCapabilitySnapshot& Capability)
{
	if (!Capability.bValid)
	{
		return Acceleration;
	}

	FVector Result = Acceleration;
	FVector2D Horizontal(Result.X, Result.Y);
	float HorizontalLimit = Capability.MaxHorizontalAccelerationCmPerSecSq;
	const float VerticalThrustAcceleration = FMath::Max(
		Capability.GravityCmPerSecSq + Result.Z, 0.0f);
	if (Capability.bHasTiltLimit)
	{
		const float TiltLimit = VerticalThrustAcceleration
			* FMath::Tan(Capability.MaxTiltRadians);
		HorizontalLimit = HorizontalLimit > 0.0f
			? FMath::Min(HorizontalLimit, TiltLimit) : TiltLimit;
	}
	if (HorizontalLimit >= 0.0f
		&& Horizontal.SizeSquared() > FMath::Square(HorizontalLimit))
	{
		Horizontal = Horizontal.GetSafeNormal() * HorizontalLimit;
		Result.X = Horizontal.X;
		Result.Y = Horizontal.Y;
	}

	if (Capability.CollectiveAuthorityN > UE_SMALL_NUMBER
		&& Capability.MassKg > UE_SMALL_NUMBER)
	{
		FVector RequiredSpecificThrust(
			Result.X, Result.Y, Capability.GravityCmPerSecSq + Result.Z);
		const float MaximumSpecificThrust = Capability.CollectiveAuthorityN
			* 100.0f / Capability.MassKg;
		RequiredSpecificThrust = RequiredSpecificThrust.GetClampedToMaxSize(
			MaximumSpecificThrust);
		Result = FVector(RequiredSpecificThrust.X, RequiredSpecificThrust.Y,
			RequiredSpecificThrust.Z - Capability.GravityCmPerSecSq);
	}
	return Result;
}

FVector FAircraftMpccController::ApplyJerkLimit(
	const FVector& PreviousAcceleration, const FVector& DesiredAcceleration,
	float DeltaTime, const FAircraftRequestedMotionLimits& Limits)
{
	FVector Delta = DesiredAcceleration - PreviousAcceleration;
	const float HorizontalStep = Limits.MaxJerkCmPerSecCubed * DeltaTime;
	FVector2D HorizontalDelta(Delta.X, Delta.Y);
	HorizontalDelta = HorizontalDelta.GetClampedToMaxSize(HorizontalStep);
	const float VerticalStep = Limits.MaxVerticalJerkCmPerSecCubed * DeltaTime;
	Delta.Z = FMath::Clamp(Delta.Z, -VerticalStep, VerticalStep);
	return PreviousAcceleration + FVector(HorizontalDelta.X, HorizontalDelta.Y, Delta.Z);
}

void FAircraftMpccController::RolloutHorizon(
	const FAircraftVehicleStateSnapshot& State,
	const FAircraftDynamicCapabilitySnapshot& Capability,
	const FAircraftRequestedMotionLimits& Limits,
	const TArray<FAircraftMotionPlanSample>& References,
	const float Dt, const int32 Steps,
	TArray<FVector>& CommandAccelerationHorizon,
	TArray<FVector>& Positions, TArray<FVector>& Velocities,
	TArray<FVector>& RealizedAccelerations, TArray<float>& ResponseAlphas)
{
	Positions[0] = State.PositionCm;
	Velocities[0] = State.VelocityCmPerSec;
	RealizedAccelerations[0] = State.AccelerationCmPerSecSq;
	for (int32 Index = 0; Index < Steps; ++Index)
	{
		CommandAccelerationHorizon[Index] = ProjectAcceleration(
			References[Index].AccelerationCmPerSecSq + ControlCorrectionHorizon[Index],
			Limits, Capability, Velocities[Index]);
		CommandAccelerationHorizon[Index] = ApplyJerkLimit(
			Index > 0 ? CommandAccelerationHorizon[Index - 1] : FilteredAccelerationCmPerSecSq,
			CommandAccelerationHorizon[Index], Dt, Limits);
		const FVector PredictedDrag = ComputeDragCompensation(
			Velocities[Index], State.BodyRotation, Capability);
		CommandAccelerationHorizon[Index] = ProjectControlAcceleration(
			CommandAccelerationHorizon[Index] + PredictedDrag, Capability) - PredictedDrag;
		const FVector GravityAcceleration(0.0f, 0.0f, Capability.GravityCmPerSecSq);
		const bool bIncreasingThrust =
			(CommandAccelerationHorizon[Index] + GravityAcceleration).SizeSquared()
				> (RealizedAccelerations[Index] + GravityAcceleration).SizeSquared();
		const float ResponseTimeSeconds = bIncreasingThrust
			? Capability.ThrustRiseResponseTimeSeconds
			: Capability.ThrustFallResponseTimeSeconds;
		ResponseAlphas[Index] = ResponseTimeSeconds > UE_SMALL_NUMBER
			? 1.0f - FMath::Exp(-Dt / ResponseTimeSeconds) : 1.0f;
		RealizedAccelerations[Index + 1] = FMath::Lerp(
			RealizedAccelerations[Index], CommandAccelerationHorizon[Index], ResponseAlphas[Index]);
		Positions[Index + 1] = Positions[Index] + Velocities[Index] * Dt
			+ 0.5f * RealizedAccelerations[Index + 1] * Dt * Dt;
		Velocities[Index + 1] = Velocities[Index]
			+ RealizedAccelerations[Index + 1] * Dt;
	}
}

FVector FAircraftMpccController::ComputeDragCompensation(
	const FVector& DesiredVelocityWorldCmPerSec, const FQuat& BodyRotation,
	const FAircraftDynamicCapabilitySnapshot& Capability)
{
	return AircraftAutopilotDynamics::ComputeDynamicsFeedForward(
		DesiredVelocityWorldCmPerSec, BodyRotation, Capability);
}

bool FAircraftMpccController::SolveVelocityIntent(
	const FAircraftVehicleStateSnapshot& State,
	const FAircraftDynamicCapabilitySnapshot& Capability,
	FAircraftTrajectoryReference& OutReference)
{
	TRACE_CPUPROFILER_EVENT_SCOPE(Aircraft_MPCC_VelocityProfile);
	const FAircraftMovementIntent& Intent = Plan.GetIntent();
	FVector TargetVelocity = AircraftAutopilotDynamics::ResolveVelocityWorld(
		Intent.Velocity.VelocityCmPerSec, Intent.Velocity.Frame, State.ControlRotation);
	const float HorizontalSpeed = FVector2D(TargetVelocity.X, TargetVelocity.Y).Size();
	const float SpeedLimit = ResolveHardLimit(Intent.Limits.CruiseSpeedCmPerSec,
		Capability.MaxHorizontalSpeedCmPerSec);
	if (SpeedLimit > 0.0f && HorizontalSpeed > SpeedLimit)
	{
		const FVector2D Clamped = FVector2D(TargetVelocity.X, TargetVelocity.Y).GetSafeNormal() * SpeedLimit;
		TargetVelocity.X = Clamped.X;
		TargetVelocity.Y = Clamped.Y;
	}
	TargetVelocity.Z = FMath::Clamp(TargetVelocity.Z,
		-Intent.Limits.MaxDescentRateCmPerSec, Intent.Limits.MaxClimbRateCmPerSec);
	if (Capability.bValid && HorizontalSpeed > UE_SMALL_NUMBER)
	{
		const FVector2D Direction = FVector2D(TargetVelocity.X, TargetVelocity.Y).GetSafeNormal();
		float LowSpeed = 0.0f;
		float HighSpeed = FVector2D(TargetVelocity.X, TargetVelocity.Y).Size();
		for (int32 Iteration = 0; Iteration < 20; ++Iteration)
		{
			const float CandidateSpeed = 0.5f * (LowSpeed + HighSpeed);
			const FVector CandidateVelocity(
				Direction.X * CandidateSpeed, Direction.Y * CandidateSpeed, TargetVelocity.Z);
			const FVector RequiredCompensation = ComputeDragCompensation(
				CandidateVelocity, State.BodyRotation, Capability);
			const FVector FeasibleCompensation = ProjectControlAcceleration(
				RequiredCompensation, Capability);
			if (FeasibleCompensation.Equals(RequiredCompensation, 0.1f))
			{
				LowSpeed = CandidateSpeed;
			}
			else
			{
				HighSpeed = CandidateSpeed;
			}
		}
		TargetVelocity.X = Direction.X * LowSpeed;
		TargetVelocity.Y = Direction.Y * LowSpeed;
	}

	const float Dt = 1.0f / RuntimeConfig.Mpcc.UpdateRateHz;
	const FVector StartingVelocity = LastReference.bValid
		? LastReference.VelocityCmPerSec
		: State.VelocityCmPerSec;
	const FVector PreviousProfileAcceleration = LastReference.bValid
		? LastVelocityProfileAccelerationCmPerSecSq
		: FVector::ZeroVector;
	FVector ProfileAcceleration = ProjectAcceleration(
		(TargetVelocity - StartingVelocity) / Dt,
		Intent.Limits, Capability, StartingVelocity);
	// S 曲线末端：当前加速度按最大 jerk 回到零仍会消耗一段速度增量。
	// 提前在该增量范围内卸载加速度，避免到达目标速度后才硬截断。
	const FVector2D HorizontalVelocityError(
		TargetVelocity.X - StartingVelocity.X,
		TargetVelocity.Y - StartingVelocity.Y);
	const FVector2D HorizontalErrorDirection = HorizontalVelocityError.GetSafeNormal();
	const float HorizontalAccelerationTowardTarget = FVector2D::DotProduct(
		FVector2D(PreviousProfileAcceleration.X, PreviousProfileAcceleration.Y),
		HorizontalErrorDirection);
	if (Intent.Limits.MaxJerkCmPerSecCubed > UE_SMALL_NUMBER
		&& HorizontalAccelerationTowardTarget > 0.0f
		&& HorizontalVelocityError.Size()
			<= FMath::Square(HorizontalAccelerationTowardTarget)
				/ (2.0f * Intent.Limits.MaxJerkCmPerSecCubed))
	{
		ProfileAcceleration.X = 0.0f;
		ProfileAcceleration.Y = 0.0f;
	}
	const float VerticalVelocityError = TargetVelocity.Z - StartingVelocity.Z;
	const float VerticalAccelerationTowardTarget = PreviousProfileAcceleration.Z
		* FMath::Sign(VerticalVelocityError);
	if (Intent.Limits.MaxVerticalJerkCmPerSecCubed > UE_SMALL_NUMBER
		&& VerticalAccelerationTowardTarget > 0.0f
		&& FMath::Abs(VerticalVelocityError)
			<= FMath::Square(VerticalAccelerationTowardTarget)
				/ (2.0f * Intent.Limits.MaxVerticalJerkCmPerSecCubed))
	{
		ProfileAcceleration.Z = 0.0f;
	}
	ProfileAcceleration = ApplyJerkLimit(
		PreviousProfileAcceleration,
		ProfileAcceleration, Dt, Intent.Limits);
	FVector CommandVelocity = StartingVelocity + ProfileAcceleration * Dt;
	const FVector RequestedVelocityDelta = TargetVelocity - StartingVelocity;
	if (FVector2D::DotProduct(
		FVector2D(RequestedVelocityDelta.X, RequestedVelocityDelta.Y),
		FVector2D(TargetVelocity.X - CommandVelocity.X,
			TargetVelocity.Y - CommandVelocity.Y)) <= 0.0f)
	{
		CommandVelocity.X = TargetVelocity.X;
		CommandVelocity.Y = TargetVelocity.Y;
	}
	if ((TargetVelocity.Z - StartingVelocity.Z)
		* (TargetVelocity.Z - CommandVelocity.Z) <= 0.0f)
	{
		CommandVelocity.Z = TargetVelocity.Z;
	}
	// 限制命令越过目标后，加速度也必须与实际生成的速度步长一致。
	// 否则参考速度已经为零却仍携带满幅制动前馈，会驱使飞机反向加速。
	ProfileAcceleration = (CommandVelocity - StartingVelocity) / Dt;
	CandidateVelocityProfileAccelerationCmPerSecSq = ProfileAcceleration;
	const FVector DragCompensation = ComputeDragCompensation(
		CommandVelocity, State.BodyRotation, Capability);
	const FVector FeasibleControlAcceleration = ProjectControlAcceleration(
		ProfileAcceleration + DragCompensation, Capability) - DragCompensation;

	OutReference.PositionCm = State.PositionCm + CommandVelocity * Dt;
	OutReference.VelocityCmPerSec = CommandVelocity;
	OutReference.AccelerationCmPerSecSq = ProfileAcceleration;
	OutReference.ControlAccelerationCmPerSecSq = FeasibleControlAcceleration;
	OutReference.DynamicsFeedForwardAccelerationCmPerSecSq = DragCompensation;
	OutReference.bPositionTrackingEnabled = false;
	const float DesiredYaw = FAircraftMotionPlan::ResolveYaw(Intent.Heading,
		State.PositionCm, TargetVelocity, State.ControlRotation.Rotator().Yaw);
	OutReference.YawDegrees = DesiredYaw;
	OutReference.YawRateDegPerSec = 0.0f;
	OutReference.YawAccelerationDegPerSecSq = 0.0f;
	OutReference.PathProgress = 0.0f;
	OutReference.RouteProgress = 0.0f;
	return true;
}

bool FAircraftMpccController::SolvePlan(
	const FAircraftVehicleStateSnapshot& State,
	const FAircraftDynamicCapabilitySnapshot& Capability,
	FAircraftTrajectoryReference& OutReference,
	const double SolveDeadlineSeconds)
{
	TRACE_CPUPROFILER_EVENT_SCOPE(Aircraft_MPCC_OptimizePlan);
	FAircraftMotionPlanSample Projection;
	const float PlanLengthCm = Plan.GetLengthCm();
	const float NominalSolveDeltaTime = 1.0f / RuntimeConfig.Mpcc.UpdateRateHz;
	const float SolveDeltaTime = FMath::Clamp(
		static_cast<float>(State.TimeSeconds - LastPlanSolveTimeSeconds),
		NominalSolveDeltaTime, RuntimeConfig.Mpcc.MaximumReferenceAgeSeconds);
	LastPlanSolveTimeSeconds = State.TimeSeconds;
	if (Plan.GetIntent().Type == EAircraftMovementIntentType::TimedTrajectory)
	{
		EstimatedPlanTimeSeconds = FMath::Clamp(
			static_cast<float>(State.TimeSeconds - PlanStartTimeSeconds),
			0.0f, Plan.GetDurationSeconds());
		if (!Plan.Evaluate(EstimatedPlanTimeSeconds, Projection))
		{
			return false;
		}
		EstimatedDistanceCm = Projection.DistanceCm;
	}
	else
	{
		const double ProjectionDeltaSeconds = bProjectionStateInitialized
			? FMath::Max(State.TimeSeconds - LastProjectionStateTimeSeconds, 0.0)
			: 0.0;
		const float MaximumVerticalSpeedCmPerSec = FMath::Max(
			Capability.MaxClimbRateCmPerSec, Capability.MaxDescentRateCmPerSec);
		const float CapabilitySpeedBoundCmPerSec = FVector2D(
			Capability.MaxHorizontalSpeedCmPerSec, MaximumVerticalSpeedCmPerSec).Size();
		const float CapabilityAccelerationBoundCmPerSecSq = FVector2D(
			Capability.MaxHorizontalAccelerationCmPerSecSq,
			Capability.MaxVerticalAccelerationCmPerSecSq).Size();
		const double ReachableDisplacementCm = CapabilitySpeedBoundCmPerSec
			* ProjectionDeltaSeconds
			+ 0.5 * CapabilityAccelerationBoundCmPerSecSq
				* FMath::Square(ProjectionDeltaSeconds);
		const float ProjectionDisplacementToleranceCm = FMath::Max(
			2.0f * RuntimeConfig.Path.ResampleSpacingCm, 100.0f);
		const bool bImpossibleStateDisplacement = bProjectionStateInitialized
			&& FVector::Distance(State.PositionCm, LastProjectionStatePositionCm)
				> ReachableDisplacementCm
					+ ProjectionDisplacementToleranceCm;
		if (!Plan.Project(State.PositionCm, EstimatedDistanceCm,
			bImpossibleStateDisplacement, Projection))
		{
			return false;
		}
		LastProjectionStatePositionCm = State.PositionCm;
		LastProjectionStateTimeSeconds = State.TimeSeconds;
		bProjectionStateInitialized = true;
		float ProjectedDistanceCm = Projection.DistanceCm;
		if (Plan.IsContinuous() && PlanLengthCm > UE_SMALL_NUMBER)
		{
			const float PreviousWrappedDistance = FMath::Fmod(
				FMath::Max(EstimatedDistanceCm, 0.0f), PlanLengthCm);
			float DistanceDelta = Projection.DistanceCm - PreviousWrappedDistance;
			if (DistanceDelta > 0.5f * PlanLengthCm)
			{
				DistanceDelta -= PlanLengthCm;
			}
			else if (DistanceDelta < -0.5f * PlanLengthCm)
			{
				DistanceDelta += PlanLengthCm;
			}
			ProjectedDistanceCm = EstimatedDistanceCm + DistanceDelta;
		}

		const float ProjectionDistanceDeltaCm = FMath::Abs(
			ProjectedDistanceCm - EstimatedDistanceCm);
		const bool bRecoveredAfterPathJump = bImpossibleStateDisplacement
			|| ProjectionDistanceDeltaCm >= FMath::Max(
				0.9f * RuntimeConfig.Path.ProjectionSearchDistanceCm, 1.0f);
		if (bRecoveredAfterPathJump)
		{
			// A local horizon is no longer meaningful after a teleport/recovery. Rebase
			// directly onto the globally recovered path point instead of rate-limiting
			// progress from a stale branch and carrying stale optimizer corrections.
			EstimatedDistanceCm = ProjectedDistanceCm;
			if (!Plan.IsContinuous())
			{
				EstimatedDistanceCm = FMath::Clamp(
					EstimatedDistanceCm, 0.0f, PlanLengthCm);
			}
			EstimatedPlanTimeSeconds = Plan.TimeAtDistance(EstimatedDistanceCm);
			ControlCorrectionHorizon.Reset();
			LastReference = {};
			PathReferenceScale = 1.0f;
		}

		const FVector ProjectionTangent = Projection.VelocityCmPerSec.GetSafeNormal();
		const float AlongTrackSpeedCmPerSec = ProjectionTangent.IsNearlyZero()
			? static_cast<float>(State.VelocityCmPerSec.Size())
			: FMath::Max(0.0f, static_cast<float>(FVector::DotProduct(
				State.VelocityCmPerSec, ProjectionTangent)));
		const float AccelerationAuthority = ResolveHardLimit(
			Plan.GetIntent().Limits.MaxAccelerationCmPerSecSq,
			Capability.MaxHorizontalAccelerationCmPerSecSq);
		const float MaximumProgressAdvanceCm = AlongTrackSpeedCmPerSec * SolveDeltaTime
			+ 0.5f * AccelerationAuthority * FMath::Square(SolveDeltaTime) + 1.0f;
		const float RequestedProgressAdvanceCm = FMath::Max(
			ProjectedDistanceCm - EstimatedDistanceCm, 0.0f);
		if (!bRecoveredAfterPathJump)
		{
			EstimatedDistanceCm += FMath::Min(
				RequestedProgressAdvanceCm, MaximumProgressAdvanceCm);
			if (!Plan.IsContinuous())
			{
				EstimatedDistanceCm = FMath::Min(EstimatedDistanceCm, PlanLengthCm);
			}
			EstimatedPlanTimeSeconds = Plan.TimeAtDistance(EstimatedDistanceCm);
		}
	}
	const FVector RawProjectionError = State.PositionCm - Projection.PositionCm;
	const FVector RawProjectionTangent = Projection.VelocityCmPerSec.GetSafeNormal();
	const FVector RawLagError = RawProjectionTangent * FVector::DotProduct(
		RawProjectionError, RawProjectionTangent);
	const float ContourErrorCm = static_cast<float>((RawProjectionError - RawLagError).Size());
	const float CorridorViolationCm = Plan.ComputeCorridorViolationCm(
		State.PositionCm, EstimatedDistanceCm);

	const int32 Steps = FMath::Clamp(RuntimeConfig.Mpcc.HorizonSteps, 2, 64);
	const float Dt = RuntimeConfig.Mpcc.HorizonSeconds / static_cast<float>(Steps);
	ReferenceScratch.SetNum(Steps + 1, EAllowShrinking::No);
	float ProgressScale = 1.0f;
	const bool bSpatialPlan = PlanLengthCm > UE_SMALL_NUMBER
		&& Plan.GetIntent().Type != EAircraftMovementIntentType::TimedTrajectory;
	if (bSpatialPlan)
	{
		FAircraftMotionPlanSample CurrentNominalReference;
		if (!Plan.Evaluate(EstimatedPlanTimeSeconds, CurrentNominalReference))
		{
			return false;
		}
		const float NominalSpeedCmPerSec = static_cast<float>(
			CurrentNominalReference.VelocityCmPerSec.Size());
		const float NormalizedContourError = ContourErrorCm
			/ FMath::Max(RuntimeConfig.Tracking.ContourErrorGovernorScaleCm, 1.0f);
		const float TargetReferenceScale = CorridorViolationCm > 0.0f
			|| Diagnostics.PredictedCorridorViolationCm > 0.0f
			? 0.0f : 1.0f / (1.0f + FMath::Square(NormalizedContourError));
		if (NominalSpeedCmPerSec > UE_SMALL_NUMBER)
		{
			const FAircraftRequestedMotionLimits& Limits = Plan.GetIntent().Limits;
			const float RequestedRate = TargetReferenceScale >= PathReferenceScale
				? ResolveHardLimit(Limits.MaxAccelerationCmPerSecSq,
					Capability.MaxHorizontalAccelerationCmPerSecSq)
				: ResolveHardLimit(Limits.MaxDecelerationCmPerSecSq,
					Capability.MaxHorizontalDecelerationCmPerSecSq);
			const float MaximumScaleStep = RequestedRate * SolveDeltaTime
				/ NominalSpeedCmPerSec;
			PathReferenceScale = FMath::Clamp(TargetReferenceScale,
				PathReferenceScale - MaximumScaleStep,
				PathReferenceScale + MaximumScaleStep);
		}
		else
		{
			PathReferenceScale = TargetReferenceScale;
		}
		ProgressScale = PathReferenceScale;
	}
	else
	{
		PathReferenceScale = 1.0f;
	}
	for (int32 Index = 0; Index <= Steps; ++Index)
	{
		if (!Plan.Evaluate(EstimatedPlanTimeSeconds
			+ ProgressScale * Dt * Index, ReferenceScratch[Index]))
		{
			return false;
		}
		if (bSpatialPlan)
		{
			ReferenceScratch[Index].VelocityCmPerSec *= ProgressScale;
			ReferenceScratch[Index].AccelerationCmPerSecSq *= FMath::Square(ProgressScale);
			ReferenceScratch[Index].YawRateDegPerSec *= ProgressScale;
		}
	}
	Diagnostics.ProgressScale = ProgressScale;

	const bool bInitializeHorizon = ControlCorrectionHorizon.Num() != Steps;
	ControlCorrectionHorizon.SetNum(Steps, EAllowShrinking::No);
	for (int32 Index = 0; Index < Steps; ++Index)
	{
		if (bInitializeHorizon || ControlCorrectionHorizon[Index].ContainsNaN())
		{
			ControlCorrectionHorizon[Index] = FVector::ZeroVector;
		}
	}
	CommandAccelerationScratch.SetNum(Steps, EAllowShrinking::No);
	PositionScratch.SetNum(Steps + 1, EAllowShrinking::No);
	VelocityScratch.SetNum(Steps + 1, EAllowShrinking::No);
	RealizedAccelerationScratch.SetNum(Steps + 1, EAllowShrinking::No);
	ResponseAlphaScratch.SetNum(Steps, EAllowShrinking::No);
	GradientScratch.SetNum(Steps, EAllowShrinking::No);
	PreviousCorrectionScratch.SetNum(Steps, EAllowShrinking::No);

	const FAircraftMpccRuntimeConfig& Mpcc = RuntimeConfig.Mpcc;
	const FAircraftRequestedMotionLimits& Limits = Plan.GetIntent().Limits;
	Diagnostics.SolverIterations = 0;
	double PreviousCost = TNumericLimits<double>::Max();
	for (int32 Iteration = 0; Iteration < Mpcc.MaxOptimizationIterations; ++Iteration)
	{
		if (FPlatformTime::Seconds() >= SolveDeadlineSeconds)
		{
			break;
		}
		RolloutHorizon(State, Capability, Limits, ReferenceScratch, Dt, Steps,
			CommandAccelerationScratch, PositionScratch, VelocityScratch,
			RealizedAccelerationScratch, ResponseAlphaScratch);
		float MaximumPredictedCorridorViolationCm = 0.0f;
		double CurrentCost = 0.0;
		for (int32 Index = 1; Index <= Steps; ++Index)
		{
			const FVector Tangent = ReferenceScratch[Index].VelocityCmPerSec.GetSafeNormal();
			const FVector PositionError = PositionScratch[Index]
				- ReferenceScratch[Index].PositionCm;
			const FVector LagError = Tangent * FVector::DotProduct(PositionError, Tangent);
			const FVector ContourError = PositionError - LagError;
			const FVector VelocityError = VelocityScratch[Index]
				- ReferenceScratch[Index].VelocityCmPerSec;
			const float TerminalPositionWeight = Index == Steps
				? Mpcc.TerminalPositionWeight : 0.0f;
			const float TerminalVelocityWeight = Index == Steps
				? Mpcc.TerminalVelocityWeight : 0.0f;
			CurrentCost += (Mpcc.ContourErrorWeight + TerminalPositionWeight)
					* ContourError.SizeSquared()
				+ (Mpcc.LagErrorWeight + TerminalPositionWeight) * LagError.SizeSquared()
				+ (Mpcc.SpeedTrackingWeight + TerminalVelocityWeight)
					* VelocityError.SizeSquared();
			const float PredictedCorridorViolationCm = Plan.ComputeCorridorViolationCm(
				PositionScratch[Index], ReferenceScratch[Index].DistanceCm);
			MaximumPredictedCorridorViolationCm = FMath::Max(
				MaximumPredictedCorridorViolationCm, PredictedCorridorViolationCm);
			CurrentCost += Mpcc.CorridorViolationWeight
				* FMath::Square(PredictedCorridorViolationCm);
		}
		for (int32 Index = 0; Index < Steps; ++Index)
		{
			CurrentCost += Mpcc.AccelerationWeight
				* ControlCorrectionHorizon[Index].SizeSquared()
				+ Mpcc.Regularization * CommandAccelerationScratch[Index].SizeSquared();
			const FVector PreviousAcceleration = Index > 0
				? CommandAccelerationScratch[Index - 1] : FilteredAccelerationCmPerSecSq;
			CurrentCost += Mpcc.JerkWeight
				* (CommandAccelerationScratch[Index] - PreviousAcceleration).SizeSquared();
		}
		if (Iteration > 0)
		{
			const double CostImprovement = PreviousCost - CurrentCost;
			if (CostImprovement < 0.0)
			{
				ControlCorrectionHorizon = PreviousCorrectionScratch;
				break;
			}
			if (CostImprovement <= OptimizationRelativeCostTolerance
				* FMath::Max(PreviousCost, 1.0))
			{
				break;
			}
		}
		PreviousCost = CurrentCost;

		FVector LambdaPosition = FVector::ZeroVector;
		FVector LambdaVelocity = FVector::ZeroVector;
		FVector LambdaAcceleration = FVector::ZeroVector;
		for (int32 Index = Steps - 1; Index >= 0; --Index)
		{
			const FVector Tangent = ReferenceScratch[Index + 1].VelocityCmPerSec.GetSafeNormal();
			const FVector PositionError = PositionScratch[Index + 1] - ReferenceScratch[Index + 1].PositionCm;
			const FVector LagError = Tangent * FVector::DotProduct(PositionError, Tangent);
			const FVector ContourError = PositionError - LagError;
			const FVector VelocityError = VelocityScratch[Index + 1] - ReferenceScratch[Index + 1].VelocityCmPerSec;
			const FVector CorridorCorrectionCm = Plan.ComputeCorridorCorrectionCm(
				PositionScratch[Index + 1], ReferenceScratch[Index + 1].DistanceCm);
			const float TerminalPositionWeight = Index == Steps - 1 ? Mpcc.TerminalPositionWeight : 0.0f;
			const float TerminalVelocityWeight = Index == Steps - 1 ? Mpcc.TerminalVelocityWeight : 0.0f;
			LambdaPosition += 2.0f * ((Mpcc.ContourErrorWeight + TerminalPositionWeight) * ContourError
				+ (Mpcc.LagErrorWeight + TerminalPositionWeight) * LagError);
			LambdaPosition -= 2.0f * Mpcc.CorridorViolationWeight * CorridorCorrectionCm;
			LambdaVelocity += 2.0f
				* (Mpcc.SpeedTrackingWeight + TerminalVelocityWeight) * VelocityError;
			const FVector AccelerationAdjoint = LambdaAcceleration
				+ 0.5f * Dt * Dt * LambdaPosition + Dt * LambdaVelocity;
			GradientScratch[Index] = ResponseAlphaScratch[Index] * AccelerationAdjoint
				+ 2.0f * Mpcc.AccelerationWeight * ControlCorrectionHorizon[Index]
				+ 2.0f * Mpcc.Regularization * CommandAccelerationScratch[Index];
			if (Mpcc.JerkWeight > 0.0f)
			{
				const FVector Previous = Index > 0
					? CommandAccelerationScratch[Index - 1] : FilteredAccelerationCmPerSecSq;
				GradientScratch[Index] += 2.0f * Mpcc.JerkWeight
					* (CommandAccelerationScratch[Index] - Previous);
				if (Index + 1 < Steps)
				{
					GradientScratch[Index] += 2.0f * Mpcc.JerkWeight
						* (CommandAccelerationScratch[Index] - CommandAccelerationScratch[Index + 1]);
				}
			}
			LambdaAcceleration = (1.0f - ResponseAlphaScratch[Index]) * AccelerationAdjoint;
			LambdaVelocity += Dt * LambdaPosition;
		}

		const float ActiveCorridorWeight = MaximumPredictedCorridorViolationCm > 0.0f
			? Mpcc.CorridorViolationWeight : 0.0f;
		const float StepSize = 1.0f / FMath::Max(1.0f,
			2.0f * (Mpcc.ContourErrorWeight + Mpcc.LagErrorWeight
				+ Mpcc.SpeedTrackingWeight + ActiveCorridorWeight) * Steps);
		double MaximumCorrectionStep = 0.0;
		PreviousCorrectionScratch = ControlCorrectionHorizon;
		for (int32 Index = 0; Index < Steps; ++Index)
		{
			const FVector CorrectionStep = GradientScratch[Index] * StepSize;
			ControlCorrectionHorizon[Index] -= CorrectionStep;
			MaximumCorrectionStep = FMath::Max(MaximumCorrectionStep,
				CorrectionStep.Size());
		}
		++Diagnostics.SolverIterations;
		if (MaximumCorrectionStep <= OptimizationCorrectionToleranceCmPerSecSq)
		{
			break;
		}
	}

	// MPCC 代价负责平滑跟踪，但“能否在终点前停住”不是可交换的软目标。
	// 当实际速度已经越过响应/jerk 感知的停止包络时，只覆盖优化结果的
	// 切向分量为最大可达制动；横向的路径/走廊修正仍然保留。
	if (!Plan.IsContinuous()
		&& Plan.GetIntent().Completion.ArrivalMode == EAircraftArrivalMode::Stop
		&& PlanLengthCm > UE_SMALL_NUMBER)
	{
		FVector BrakingTangent = Projection.VelocityCmPerSec.GetSafeNormal();
		if (BrakingTangent.IsNearlyZero())
		{
			BrakingTangent = ReferenceScratch[0].VelocityCmPerSec.GetSafeNormal();
		}
		if (BrakingTangent.IsNearlyZero())
		{
			BrakingTangent = State.VelocityCmPerSec.GetSafeNormal();
		}
		if (!BrakingTangent.IsNearlyZero())
		{
			const float BrakingReserveScale =
				1.0f - RuntimeConfig.Timing.BrakingReserveFraction;
			const float HorizontalDeceleration = ResolveHardLimit(
				Limits.MaxDecelerationCmPerSecSq,
				Capability.MaxHorizontalDecelerationCmPerSecSq)
				* BrakingReserveScale;
			const float VerticalDeceleration = ResolveHardLimit(
				Limits.MaxVerticalAccelerationCmPerSecSq,
				Capability.MaxVerticalAccelerationCmPerSecSq)
				* BrakingReserveScale;
			const float TangentialDeceleration =
				AircraftAutopilotDynamics::ResolveTangentialLimit(
					BrakingTangent, HorizontalDeceleration, VerticalDeceleration);
			const float TangentialJerk =
				AircraftAutopilotDynamics::ResolveTangentialLimit(
					BrakingTangent,
					Limits.MaxJerkCmPerSecCubed,
					Limits.MaxVerticalJerkCmPerSecCubed);
			const float BrakingDelaySeconds =
				AircraftAutopilotDynamics::ComputeBrakingDelaySeconds(
					TangentialDeceleration, TangentialJerk, Capability);
			const float RemainingDistanceCm = FMath::Max(
				PlanLengthCm - Projection.DistanceCm, 0.0f);
			const float ActualAlongTrackSpeedCmPerSec = FMath::Max(
				static_cast<float>(FVector::DotProduct(
					State.VelocityCmPerSec, BrakingTangent)), 0.0f);
			const float RequiredStoppingDistanceCm =
				AircraftAutopilotDynamics::ComputeStoppingDistanceCm(
					ActualAlongTrackSpeedCmPerSec,
					TangentialDeceleration,
					BrakingDelaySeconds);
			if (RequiredStoppingDistanceCm
				> RemainingDistanceCm + UE_KINDA_SMALL_NUMBER)
			{
				for (int32 Index = 0; Index < Steps; ++Index)
				{
					FVector Tangent = ReferenceScratch[Index].VelocityCmPerSec.GetSafeNormal();
					if (Tangent.IsNearlyZero())
					{
						Tangent = BrakingTangent;
					}
					const float StepDeceleration =
						AircraftAutopilotDynamics::ResolveTangentialLimit(
							Tangent, HorizontalDeceleration, VerticalDeceleration);
					const FVector PlannedAcceleration =
						ReferenceScratch[Index].AccelerationCmPerSecSq
							+ ControlCorrectionHorizon[Index];
					ControlCorrectionHorizon[Index] += Tangent
						* (-StepDeceleration - static_cast<float>(
							FVector::DotProduct(PlannedAcceleration, Tangent)));
				}
			}
		}
	}

	// 最后一次梯度更新发生在约束投影之后；发布前必须重新滚动一次，
	// 保证真正输出的第一步仍满足 jerk、倾角、总推力和阻力补偿约束。
	RolloutHorizon(State, Capability, Limits, ReferenceScratch, Dt, Steps,
		CommandAccelerationScratch, PositionScratch, VelocityScratch,
		RealizedAccelerationScratch, ResponseAlphaScratch);
	Diagnostics.PredictedCorridorViolationCm = 0.0f;
	for (int32 Index = 1; Index <= Steps; ++Index)
	{
		Diagnostics.PredictedCorridorViolationCm = FMath::Max(
			Diagnostics.PredictedCorridorViolationCm,
			Plan.ComputeCorridorViolationCm(
				PositionScratch[Index], ReferenceScratch[Index].DistanceCm));
	}

	OutReference.PositionCm = ReferenceScratch[0].PositionCm;
	OutReference.VelocityCmPerSec = ReferenceScratch[0].VelocityCmPerSec;
	const FVector DragCompensation = ComputeDragCompensation(
		ReferenceScratch[0].VelocityCmPerSec, State.BodyRotation, Capability);
	OutReference.AccelerationCmPerSecSq = ReferenceScratch[0].AccelerationCmPerSecSq;
	OutReference.ControlAccelerationCmPerSecSq = ProjectControlAcceleration(
		CommandAccelerationScratch[0] + DragCompensation, Capability) - DragCompensation;
	OutReference.DynamicsFeedForwardAccelerationCmPerSecSq = DragCompensation;
	OutReference.bPositionTrackingEnabled = true;
	OutReference.YawDegrees = ReferenceScratch[0].YawDegrees;
	OutReference.YawRateDegPerSec = 0.0f;
	OutReference.YawAccelerationDegPerSecSq = 0.0f;
	if (Plan.GetIntent().Type == EAircraftMovementIntentType::TimedTrajectory)
	{
		OutReference.PathProgress = FMath::Clamp(EstimatedPlanTimeSeconds
			/ FMath::Max(Plan.GetDurationSeconds(), UE_SMALL_NUMBER), 0.0f, 1.0f);
		OutReference.RouteProgress = 0.0f;
	}
	else
	{
		OutReference.PathProgress = PlanLengthCm > UE_SMALL_NUMBER
			? (Plan.IsContinuous()
				? FMath::Fmod(EstimatedDistanceCm, PlanLengthCm) / PlanLengthCm
				: FMath::Clamp(EstimatedDistanceCm / PlanLengthCm, 0.0f, 1.0f))
			: 0.0f;
		OutReference.RouteProgress = Plan.GetRouteLengthCm() > UE_SMALL_NUMBER
			? Plan.GetRouteDistanceCm(EstimatedDistanceCm) / Plan.GetRouteLengthCm()
			: 0.0f;
	}
	Diagnostics.ContourErrorCm = ContourErrorCm;
	Diagnostics.CorridorViolationCm = CorridorViolationCm;
	Diagnostics.bCorridorViolated = CorridorViolationCm > 0.0f;
	Diagnostics.PathTrackingState = Diagnostics.bCorridorViolated
		? EAircraftPathTrackingState::CorridorRecovery
		: (Diagnostics.PredictedCorridorViolationCm > 0.0f
			? EAircraftPathTrackingState::CorridorConstrained
			: (ProgressScale < 1.0f - UE_KINDA_SMALL_NUMBER
				? EAircraftPathTrackingState::ContourLimited
				: EAircraftPathTrackingState::Nominal));
	Diagnostics.LagErrorCm = static_cast<float>(FVector::DotProduct(
		RawProjectionError, RawProjectionTangent));
	const FVector LastCorrection = ControlCorrectionHorizon.Last();
	for (int32 Index = 0; Index + 1 < Steps; ++Index)
	{
		ControlCorrectionHorizon[Index] = ControlCorrectionHorizon[Index + 1];
	}
	ControlCorrectionHorizon.Last() = LastCorrection;
	return true;
}

void FAircraftMpccController::RebaseTime(const double TimeSeconds)
{
	const double PausedDuration = FMath::Max(TimeSeconds - LastPlanSolveTimeSeconds, 0.0);
	PlanStartTimeSeconds += PausedDuration;
	LastPlanSolveTimeSeconds = TimeSeconds;
	if (bProjectionStateInitialized)
	{
		LastProjectionStateTimeSeconds = TimeSeconds;
	}
	if (LastReference.bValid)
	{
		LastReference.GeneratedAtSeconds += PausedDuration;
		LastReference.ValidUntilSeconds += PausedDuration;
	}
	if (NextSolveTimeSeconds > -DBL_MAX)
	{
		NextSolveTimeSeconds += PausedDuration;
	}
}

bool FAircraftMpccController::Update(
	const FAircraftVehicleStateSnapshot& State,
	const FAircraftDynamicCapabilitySnapshot& Capability,
	FAircraftTrajectoryReference& OutReference)
{
	OutReference = {};
	if (!Plan.IsValid())
	{
		return false;
	}
	if (!RefreshPlanForCapability(State, Capability))
	{
		return false;
	}
	// 实测加速度（物理差分）噪声大，作 jerk 种子会直接把噪声灌进 50Hz 控制指令。
	// 一阶低通后再作视界第一步的 jerk 限制基准。
	{
		const double DeltaSeconds = FMath::Max(
			State.TimeSeconds - LastFilterUpdateTimeSeconds, 0.0);
		const float Alpha = FMath::Clamp(
			static_cast<float>(DeltaSeconds / (1.0 / (2.0 * PI * JerkSeedFilterCutoffHz)
				+ DeltaSeconds)),
			0.0f, 1.0f);
		if (DeltaSeconds > 0.0)
		{
			if (!bFilterInitialized && !State.AccelerationCmPerSecSq.ContainsNaN())
			{
				FilteredAccelerationCmPerSecSq = State.AccelerationCmPerSecSq;
				bFilterInitialized = true;
			}
			else
			{
				FilteredAccelerationCmPerSecSq += (State.AccelerationCmPerSecSq
					- FilteredAccelerationCmPerSecSq) * Alpha;
			}
			LastFilterUpdateTimeSeconds = State.TimeSeconds;
		}
	}
	const double SolveInterval = 1.0 / RuntimeConfig.Mpcc.UpdateRateHz;
	if (LastReference.IsFresh(State.TimeSeconds)
		&& NextSolveTimeSeconds > -DBL_MAX
		&& State.TimeSeconds + UE_DOUBLE_SMALL_NUMBER < NextSolveTimeSeconds)
	{
		OutReference = LastReference;
		Diagnostics.bReferenceFresh = true;
		Diagnostics.bSolverFailed = false;
		return true;
	}

	const double StartSeconds = FPlatformTime::Seconds();
	const double SolveDeadlineSeconds = StartSeconds
		+ RuntimeConfig.Mpcc.SolveTimeBudgetMilliseconds * 0.001;
	FAircraftTrajectoryReference Candidate;
	bool bSolved = false;
	if (Plan.GetIntent().Type == EAircraftMovementIntentType::Velocity)
	{
		Diagnostics.SolverIterations = 0;
		bSolved = SolveVelocityIntent(State, Capability, Candidate);
	}
	else
	{
		bSolved = SolvePlan(State, Capability, Candidate, SolveDeadlineSeconds);
	}
	const double ElapsedMilliseconds = (FPlatformTime::Seconds() - StartSeconds) * 1000.0;
	Diagnostics.LastSolveMilliseconds = ElapsedMilliseconds;
	Diagnostics.MaximumSolveMilliseconds = FMath::Max(
		Diagnostics.MaximumSolveMilliseconds, ElapsedMilliseconds);
	if (!bSolved)
	{
		++Diagnostics.ConsecutiveFailures;
		Diagnostics.bReferenceFresh = false;
		Diagnostics.bSolverFailed = Diagnostics.ConsecutiveFailures
			>= RuntimeConfig.Mpcc.MaxConsecutiveFailures;
		if (Diagnostics.ConsecutiveFailures < RuntimeConfig.Mpcc.MaxConsecutiveFailures
			&& LastReference.IsFresh(State.TimeSeconds))
		{
			OutReference = LastReference;
			Diagnostics.bReferenceFresh = true;
			Diagnostics.bSolverFailed = false;
			return true;
		}
		return false;
	}

	Diagnostics.ConsecutiveFailures = 0;
	Diagnostics.bSolverFailed = false;
	Candidate.IntentId = ActiveIntentId;
	Candidate.IntentRevision = IntentRevision;
	Candidate.PlanRevision = PlanRevision;
	Candidate.StateSequence = State.Sequence;
	Candidate.GeneratedAtSeconds = State.TimeSeconds;
	Candidate.ValidUntilSeconds = State.TimeSeconds + RuntimeConfig.Mpcc.MaximumReferenceAgeSeconds;
	Candidate.YawRateLimitDegPerSec = Plan.GetIntent().Limits.MaxYawRateDegPerSec;
	Candidate.bValid = true;
	if (NextSolveTimeSeconds <= -DBL_MAX)
	{
		NextSolveTimeSeconds = State.TimeSeconds + SolveInterval;
	}
	else
	{
		do
		{
			NextSolveTimeSeconds += SolveInterval;
		}
		while (NextSolveTimeSeconds <= State.TimeSeconds + UE_DOUBLE_SMALL_NUMBER);
	}
	LastReference = Candidate;
	if (Plan.GetIntent().Type == EAircraftMovementIntentType::Velocity)
	{
		LastVelocityProfileAccelerationCmPerSecSq =
			CandidateVelocityProfileAccelerationCmPerSecSq;
	}
	OutReference = Candidate;
	Diagnostics.PlanRevision = PlanRevision;
	Diagnostics.bReferenceFresh = true;
	return true;
}

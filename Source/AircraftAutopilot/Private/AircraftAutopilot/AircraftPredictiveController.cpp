#include "AircraftAutopilot/AircraftPredictiveController.h"

#include "HAL/PlatformTime.h"

namespace
{
	float ResolvePositiveLimit(float Requested, float Available)
	{
		if (Requested <= 0.0f) return Available;
		if (Available <= 0.0f) return Requested;
		return FMath::Min(Requested, Available);
	}
}

void FAircraftPredictiveController::Reset()
{
	Plan.Reset();
	RuntimeConfig = {};
	Diagnostics = {};
	LastReference = {};
	LastVelocityProfileAccelerationCmPerSecSq = FVector::ZeroVector;
	CandidateVelocityProfileAccelerationCmPerSecSq = FVector::ZeroVector;
	ControlCorrectionHorizon.Reset();
	IntentRevision = 0;
	ActiveIntentId = 0;
	PlanRevision = 0;
	EstimatedPlanTimeSeconds = 0.0f;
	EstimatedDistanceCm = 0.0f;
	PathReferenceScale = 1.0f;
	PlanStartTimeSeconds = 0.0;
	LastPlanSolveTimeSeconds = 0.0;
	NextSolveTimeSeconds = -DBL_MAX;
}

bool FAircraftPredictiveController::SetIntent(
	const FAircraftMovementIntent& Intent, int64 InIntentId, uint64 InIntentRevision,
	const FAircraftAutopilotRuntimeConfig& Config,
	const FAircraftVehicleStateSnapshot& State,
	const FAircraftDynamicCapabilitySnapshot& Capability)
{
	const bool bPreserveVelocityProfile = Plan.IsValid()
		&& ActiveIntentId == InIntentId
		&& Plan.GetIntent().Type == EAircraftMovementIntentType::Velocity
		&& Intent.Type == EAircraftMovementIntentType::Velocity;
	RuntimeConfig = Config;
	IntentRevision = InIntentRevision;
	ActiveIntentId = InIntentId;
	Diagnostics = {};
	Diagnostics.ActiveIntentId = InIntentId;
	Diagnostics.IntentRevision = InIntentRevision;
	ControlCorrectionHorizon.Reset();
	EstimatedPlanTimeSeconds = 0.0f;
	EstimatedDistanceCm = 0.0f;
	PathReferenceScale = 1.0f;
	PlanStartTimeSeconds = State.TimeSeconds;
	LastPlanSolveTimeSeconds = State.TimeSeconds;
	if (!bPreserveVelocityProfile)
	{
		LastReference = {};
		LastVelocityProfileAccelerationCmPerSecSq = FVector::ZeroVector;
		CandidateVelocityProfileAccelerationCmPerSecSq = FVector::ZeroVector;
	}
	// 新 revision 必须立即求解，但同一 Velocity handle 不丢弃速度/加速度连续状态。
	NextSolveTimeSeconds = -DBL_MAX;
	const bool bBuilt = Plan.Build(Intent, Config, State, Capability);
	if (bBuilt)
	{
		++PlanRevision;
		if (Intent.Type == EAircraftMovementIntentType::Route
			|| Intent.Type == EAircraftMovementIntentType::Orbit)
		{
			FAircraftMotionPlanSample InitialProjection;
			if (Plan.Project(State.PositionCm, 0.0f, InitialProjection))
			{
				EstimatedDistanceCm = InitialProjection.DistanceCm;
				EstimatedPlanTimeSeconds = Plan.TimeAtDistance(EstimatedDistanceCm);
			}
		}
	}
	Diagnostics.bPlanValid = bBuilt;
	return bBuilt;
}

FVector FAircraftPredictiveController::ProjectAcceleration(
	const FVector& Acceleration,
	const FAircraftRequestedMotionLimits& Limits,
	const FAircraftDynamicCapabilitySnapshot& Capability,
	const FVector& VelocityCmPerSec)
{
	FVector Result = Acceleration;
	const bool bHorizontalBraking = FVector2D::DotProduct(
		FVector2D(Result.X, Result.Y), FVector2D(VelocityCmPerSec.X, VelocityCmPerSec.Y)) < 0.0f;
	const float HorizontalLimit = ResolvePositiveLimit(
		bHorizontalBraking ? Limits.MaxDecelerationCmPerSecSq : Limits.MaxAccelerationCmPerSecSq,
		Capability.MaxHorizontalAccelerationCmPerSecSq);
	const FVector2D Horizontal(Result.X, Result.Y);
	if (HorizontalLimit > 0.0f && Horizontal.SizeSquared() > FMath::Square(HorizontalLimit))
	{
		const FVector2D Clamped = Horizontal.GetSafeNormal() * HorizontalLimit;
		Result.X = Clamped.X;
		Result.Y = Clamped.Y;
	}
	const float VerticalLimit = ResolvePositiveLimit(
		Limits.MaxVerticalAccelerationCmPerSecSq, Capability.MaxVerticalAccelerationCmPerSecSq);
	if (VerticalLimit > 0.0f)
	{
		Result.Z = FMath::Clamp(Result.Z, -VerticalLimit, VerticalLimit);
	}
	return Result;
}

FVector FAircraftPredictiveController::ProjectControlAcceleration(
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
	if (Capability.MaxTiltRadians > UE_SMALL_NUMBER)
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

FVector FAircraftPredictiveController::ApplyJerkLimit(
	const FVector& PreviousAcceleration, const FVector& DesiredAcceleration,
	float DeltaTime, const FAircraftRequestedMotionLimits& Limits)
{
	FVector Delta = DesiredAcceleration - PreviousAcceleration;
	const float HorizontalStep = Limits.MaxJerkCmPerSecCubed * DeltaTime;
	FVector2D HorizontalDelta(Delta.X, Delta.Y);
	if (HorizontalStep > 0.0f)
	{
		HorizontalDelta = HorizontalDelta.GetClampedToMaxSize(HorizontalStep);
	}
	const float VerticalStep = Limits.MaxVerticalJerkCmPerSecCubed * DeltaTime;
	if (VerticalStep > 0.0f)
	{
		Delta.Z = FMath::Clamp(Delta.Z, -VerticalStep, VerticalStep);
	}
	return PreviousAcceleration + FVector(HorizontalDelta.X, HorizontalDelta.Y, Delta.Z);
}

void FAircraftPredictiveController::ApplyYawConstraints(
	const FAircraftVehicleStateSnapshot& State,
	const FAircraftRequestedMotionLimits& Limits, float DeltaTime,
	float DesiredYawDegrees,
	FAircraftTrajectoryReference& InOutReference) const
{
	const float StartingYaw = State.ControlRotation.Rotator().Yaw;
	const FVector AngularVelocityWorld = State.BodyRotation.RotateVector(
		State.AngularVelocityBodyRadPerSec);
	const float PreviousYawRate = FMath::RadiansToDegrees(AngularVelocityWorld.Z);
	const float PreviousYawAcceleration = LastReference.bValid
		? LastReference.YawAccelerationDegPerSecSq : 0.0f;
	const float YawError = FMath::FindDeltaAngleDegrees(StartingYaw, DesiredYawDegrees);
	const float TrackingAlpha = 1.0f - FMath::Exp(
		-FMath::Max(RuntimeConfig.Mpcc.YawTrackingWeight, 0.0f));
	const float RequestedRate = FMath::Clamp(
		YawError * TrackingAlpha / FMath::Max(DeltaTime, UE_SMALL_NUMBER),
		-Limits.MaxYawRateDegPerSec, Limits.MaxYawRateDegPerSec);
	float YawAcceleration = (RequestedRate - PreviousYawRate)
		/ FMath::Max(DeltaTime, UE_SMALL_NUMBER);
	YawAcceleration = FMath::Clamp(YawAcceleration,
		-Limits.MaxYawAccelerationDegPerSecSq, Limits.MaxYawAccelerationDegPerSecSq);
	if (Limits.MaxYawJerkDegPerSecCubed > UE_SMALL_NUMBER)
	{
		const float MaximumAccelerationStep = Limits.MaxYawJerkDegPerSecCubed * DeltaTime;
		YawAcceleration = FMath::Clamp(YawAcceleration,
			PreviousYawAcceleration - MaximumAccelerationStep,
			PreviousYawAcceleration + MaximumAccelerationStep);
	}
	InOutReference.YawAccelerationDegPerSecSq = YawAcceleration;
	InOutReference.YawRateDegPerSec = FMath::Clamp(
		PreviousYawRate + YawAcceleration * DeltaTime,
		-Limits.MaxYawRateDegPerSec, Limits.MaxYawRateDegPerSec);
	InOutReference.YawDegrees = FRotator::NormalizeAxis(StartingYaw
		+ 0.5f * (PreviousYawRate + InOutReference.YawRateDegPerSec) * DeltaTime);
	if (YawError * FMath::FindDeltaAngleDegrees(
		InOutReference.YawDegrees, DesiredYawDegrees) <= 0.0f)
	{
		InOutReference.YawDegrees = FRotator::NormalizeAxis(DesiredYawDegrees);
		InOutReference.YawRateDegPerSec = 0.0f;
		InOutReference.YawAccelerationDegPerSecSq = 0.0f;
	}
}

FVector FAircraftPredictiveController::ComputeDragCompensation(
	const FVector& DesiredVelocityWorldCmPerSec, const FQuat& BodyRotation,
	const FAircraftDynamicCapabilitySnapshot& Capability)
{
	FVector Compensation = Capability.LinearDampingPerSecond * DesiredVelocityWorldCmPerSec;
	if (!Capability.bHasExplicitAerodynamics || Capability.MassKg <= UE_SMALL_NUMBER)
	{
		return Compensation;
	}
	const FVector VelocityBodyMps = BodyRotation.UnrotateVector(DesiredVelocityWorldCmPerSec) * 0.01f;
	const FVector QuadraticCoefficient = Capability.DragAreaCoefficientBodyM2
		* (0.5f * Capability.AirDensityKgPerM3);
	const FVector RequiredForceBodyN = Capability.LinearDragBodyNsPerM * VelocityBodyMps
		+ QuadraticCoefficient * VelocityBodyMps.GetAbs() * VelocityBodyMps;
	return Compensation + BodyRotation.RotateVector(RequiredForceBodyN)
		* (100.0f / Capability.MassKg);
}

bool FAircraftPredictiveController::SolveVelocityIntent(
	const FAircraftVehicleStateSnapshot& State,
	const FAircraftDynamicCapabilitySnapshot& Capability,
	FAircraftTrajectoryReference& OutReference)
{
	const FAircraftMovementIntent& Intent = Plan.GetIntent();
	FVector TargetVelocity = Intent.Velocity.VelocityCmPerSec;
	if (Intent.Velocity.Frame == EAircraftVelocityFrame::ControlHeading)
	{
		TargetVelocity = State.ControlRotation.RotateVector(TargetVelocity);
	}
	const float HorizontalSpeed = FVector2D(TargetVelocity.X, TargetVelocity.Y).Size();
	const float SpeedLimit = ResolvePositiveLimit(Intent.Limits.CruiseSpeedCmPerSec,
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
	ApplyYawConstraints(State, Intent.Limits, Dt, DesiredYaw, OutReference);
	OutReference.PathProgress = 0.0f;
	return true;
}

bool FAircraftPredictiveController::SolvePlan(
	const FAircraftVehicleStateSnapshot& State,
	const FAircraftDynamicCapabilitySnapshot& Capability,
	FAircraftTrajectoryReference& OutReference)
{
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
		if (!Plan.Project(State.PositionCm, EstimatedDistanceCm, Projection))
		{
			return false;
		}
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

		const FVector ProjectionTangent = Projection.VelocityCmPerSec.GetSafeNormal();
		const float AlongTrackSpeedCmPerSec = ProjectionTangent.IsNearlyZero()
			? static_cast<float>(State.VelocityCmPerSec.Size())
			: FMath::Max(0.0f, static_cast<float>(FVector::DotProduct(
				State.VelocityCmPerSec, ProjectionTangent)));
		const float AccelerationAuthority = ResolvePositiveLimit(
			Plan.GetIntent().Limits.MaxAccelerationCmPerSecSq,
			Capability.MaxHorizontalAccelerationCmPerSecSq);
		const float MaximumProgressAdvanceCm = AlongTrackSpeedCmPerSec * SolveDeltaTime
			+ 0.5f * AccelerationAuthority * FMath::Square(SolveDeltaTime) + 1.0f;
		const float RequestedProgressAdvanceCm = FMath::Max(
			ProjectedDistanceCm - EstimatedDistanceCm, 0.0f);
		EstimatedDistanceCm += FMath::Min(
			RequestedProgressAdvanceCm, MaximumProgressAdvanceCm);
		if (!Plan.IsContinuous())
		{
			EstimatedDistanceCm = FMath::Min(EstimatedDistanceCm, PlanLengthCm);
		}
		EstimatedPlanTimeSeconds = Plan.TimeAtDistance(EstimatedDistanceCm);
	}
	const FVector RawProjectionError = State.PositionCm - Projection.PositionCm;
	const FVector RawProjectionTangent = Projection.VelocityCmPerSec.GetSafeNormal();
	const FVector RawLagError = RawProjectionTangent * FVector::DotProduct(
		RawProjectionError, RawProjectionTangent);
	const float ContourErrorCm = static_cast<float>((RawProjectionError - RawLagError).Size());

	const int32 Steps = FMath::Clamp(RuntimeConfig.Mpcc.HorizonSteps, 2, 64);
	const float Dt = RuntimeConfig.Mpcc.HorizonSeconds / static_cast<float>(Steps);
	TArray<FAircraftMotionPlanSample> References;
	References.SetNum(Steps + 1);
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
			/ FMath::Max(RuntimeConfig.Path.ResampleSpacingCm, 1.0f);
		const float TargetReferenceScale = 1.0f
			/ (1.0f + FMath::Square(NormalizedContourError));
		if (NominalSpeedCmPerSec > UE_SMALL_NUMBER)
		{
			const FAircraftRequestedMotionLimits& Limits = Plan.GetIntent().Limits;
			const float RequestedRate = TargetReferenceScale >= PathReferenceScale
				? ResolvePositiveLimit(Limits.MaxAccelerationCmPerSecSq,
					Capability.MaxHorizontalAccelerationCmPerSecSq)
				: ResolvePositiveLimit(Limits.MaxDecelerationCmPerSecSq,
					Capability.MaxHorizontalAccelerationCmPerSecSq);
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
			+ ProgressScale * Dt * Index, References[Index]))
		{
			return false;
		}
		if (bSpatialPlan)
		{
			References[Index].VelocityCmPerSec *= ProgressScale;
			References[Index].AccelerationCmPerSecSq *= FMath::Square(ProgressScale);
			References[Index].YawRateDegPerSec *= ProgressScale;
		}
	}
	Diagnostics.ProgressScale = ProgressScale;

	const bool bInitializeHorizon = ControlCorrectionHorizon.Num() != Steps;
	ControlCorrectionHorizon.SetNum(Steps);
	for (int32 Index = 0; Index < Steps; ++Index)
	{
		if (bInitializeHorizon || ControlCorrectionHorizon[Index].ContainsNaN())
		{
			ControlCorrectionHorizon[Index] = FVector::ZeroVector;
		}
	}
	TArray<FVector> CommandAccelerationHorizon;
	CommandAccelerationHorizon.SetNum(Steps);
	TArray<FVector> Positions;
	TArray<FVector> Velocities;
	Positions.SetNum(Steps + 1);
	Velocities.SetNum(Steps + 1);
	TArray<FVector> RealizedAccelerations;
	RealizedAccelerations.SetNum(Steps + 1);
	TArray<FVector> Gradient;
	Gradient.SetNum(Steps);

	const FAircraftMpccRuntimeConfig& Mpcc = RuntimeConfig.Mpcc;
	const FAircraftRequestedMotionLimits& Limits = Plan.GetIntent().Limits;
	const float ResponseAlpha = Capability.RotorResponseTimeSeconds > UE_SMALL_NUMBER
		? 1.0f - FMath::Exp(-Dt / Capability.RotorResponseTimeSeconds)
		: 1.0f;
	for (int32 Iteration = 0; Iteration < Mpcc.MaxOptimizationIterations; ++Iteration)
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
				Index > 0 ? CommandAccelerationHorizon[Index - 1] : State.AccelerationCmPerSecSq,
				CommandAccelerationHorizon[Index], Dt, Limits);
			const FVector DragCompensation = ComputeDragCompensation(
				Velocities[Index], State.BodyRotation, Capability);
			CommandAccelerationHorizon[Index] = ProjectControlAcceleration(
				CommandAccelerationHorizon[Index] + DragCompensation, Capability)
				- DragCompensation;
			RealizedAccelerations[Index + 1] = FMath::Lerp(
				RealizedAccelerations[Index], CommandAccelerationHorizon[Index], ResponseAlpha);
			Positions[Index + 1] = Positions[Index] + Velocities[Index] * Dt
				+ 0.5f * RealizedAccelerations[Index + 1] * Dt * Dt;
			Velocities[Index + 1] = Velocities[Index]
				+ RealizedAccelerations[Index + 1] * Dt;
		}

		FVector LambdaPosition = FVector::ZeroVector;
		FVector LambdaVelocity = FVector::ZeroVector;
		FVector LambdaAcceleration = FVector::ZeroVector;
		for (int32 Index = Steps - 1; Index >= 0; --Index)
		{
			const FVector Tangent = References[Index + 1].VelocityCmPerSec.GetSafeNormal();
			const FVector PositionError = Positions[Index + 1] - References[Index + 1].PositionCm;
			const FVector LagError = Tangent * FVector::DotProduct(PositionError, Tangent);
			const FVector ContourError = PositionError - LagError;
			const FVector VelocityError = Velocities[Index + 1] - References[Index + 1].VelocityCmPerSec;
			const float TerminalPositionWeight = Index == Steps - 1 ? Mpcc.TerminalPositionWeight : 0.0f;
			const float TerminalVelocityWeight = Index == Steps - 1 ? Mpcc.TerminalVelocityWeight : 0.0f;
			LambdaPosition += 2.0f * ((Mpcc.ContourErrorWeight + TerminalPositionWeight) * ContourError
				+ (Mpcc.LagErrorWeight + TerminalPositionWeight) * LagError);
			LambdaVelocity += 2.0f
				* (Mpcc.SpeedTrackingWeight + TerminalVelocityWeight) * VelocityError;
			const FVector AccelerationAdjoint = LambdaAcceleration
				+ 0.5f * Dt * Dt * LambdaPosition + Dt * LambdaVelocity;
			Gradient[Index] = ResponseAlpha * AccelerationAdjoint
				+ 2.0f * Mpcc.AccelerationWeight * ControlCorrectionHorizon[Index]
				+ 2.0f * Mpcc.Regularization * CommandAccelerationHorizon[Index];
			if (Mpcc.JerkWeight > 0.0f)
			{
				const FVector Previous = Index > 0
					? CommandAccelerationHorizon[Index - 1] : State.AccelerationCmPerSecSq;
				Gradient[Index] += 2.0f * Mpcc.JerkWeight
					* (CommandAccelerationHorizon[Index] - Previous);
				if (Index + 1 < Steps)
				{
					Gradient[Index] += 2.0f * Mpcc.JerkWeight
						* (CommandAccelerationHorizon[Index] - CommandAccelerationHorizon[Index + 1]);
				}
			}
			LambdaAcceleration = (1.0f - ResponseAlpha) * AccelerationAdjoint;
			LambdaVelocity += Dt * LambdaPosition;
		}

		const float StepSize = 1.0f / FMath::Max(1.0f,
			2.0f * (Mpcc.ContourErrorWeight + Mpcc.LagErrorWeight
				+ Mpcc.SpeedTrackingWeight) * Steps);
		for (int32 Index = 0; Index < Steps; ++Index)
		{
			ControlCorrectionHorizon[Index] -= Gradient[Index] * StepSize;
		}
	}

	// 最后一次梯度更新发生在约束投影之后；发布前必须重新滚动一次，
	// 保证真正输出的第一步仍满足 jerk、倾角、总推力和阻力补偿约束。
	Positions[0] = State.PositionCm;
	Velocities[0] = State.VelocityCmPerSec;
	RealizedAccelerations[0] = State.AccelerationCmPerSecSq;
	for (int32 Index = 0; Index < Steps; ++Index)
	{
		CommandAccelerationHorizon[Index] = ProjectAcceleration(
			References[Index].AccelerationCmPerSecSq + ControlCorrectionHorizon[Index],
			Limits, Capability, Velocities[Index]);
		CommandAccelerationHorizon[Index] = ApplyJerkLimit(
			Index > 0 ? CommandAccelerationHorizon[Index - 1] : State.AccelerationCmPerSecSq,
			CommandAccelerationHorizon[Index], Dt, Limits);
		const FVector PredictedDrag = ComputeDragCompensation(
			Velocities[Index], State.BodyRotation, Capability);
		CommandAccelerationHorizon[Index] = ProjectControlAcceleration(
			CommandAccelerationHorizon[Index] + PredictedDrag, Capability) - PredictedDrag;
		RealizedAccelerations[Index + 1] = FMath::Lerp(
			RealizedAccelerations[Index], CommandAccelerationHorizon[Index], ResponseAlpha);
		Positions[Index + 1] = Positions[Index] + Velocities[Index] * Dt
			+ 0.5f * RealizedAccelerations[Index + 1] * Dt * Dt;
		Velocities[Index + 1] = Velocities[Index]
			+ RealizedAccelerations[Index + 1] * Dt;
	}

	OutReference.PositionCm = References[0].PositionCm;
	OutReference.VelocityCmPerSec = References[0].VelocityCmPerSec;
	const FVector DragCompensation = ComputeDragCompensation(
		References[0].VelocityCmPerSec, State.BodyRotation, Capability);
	OutReference.AccelerationCmPerSecSq = References[0].AccelerationCmPerSecSq;
	OutReference.ControlAccelerationCmPerSecSq = ProjectControlAcceleration(
		CommandAccelerationHorizon[0] + DragCompensation, Capability) - DragCompensation;
	OutReference.DynamicsFeedForwardAccelerationCmPerSecSq = DragCompensation;
	OutReference.bPositionTrackingEnabled = true;
	ApplyYawConstraints(State, Limits, 1.0f / Mpcc.UpdateRateHz,
		References[0].YawDegrees, OutReference);
	if (Plan.GetIntent().Type == EAircraftMovementIntentType::TimedTrajectory)
	{
		OutReference.PathProgress = FMath::Clamp(EstimatedPlanTimeSeconds
			/ FMath::Max(Plan.GetDurationSeconds(), UE_SMALL_NUMBER), 0.0f, 1.0f);
	}
	else
	{
		OutReference.PathProgress = PlanLengthCm > UE_SMALL_NUMBER
			? (Plan.IsContinuous()
				? FMath::Fmod(EstimatedDistanceCm, PlanLengthCm) / PlanLengthCm
				: FMath::Clamp(EstimatedDistanceCm / PlanLengthCm, 0.0f, 1.0f))
			: 0.0f;
	}
	Diagnostics.ContourErrorCm = ContourErrorCm;
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

bool FAircraftPredictiveController::Update(
	const FAircraftVehicleStateSnapshot& State,
	const FAircraftDynamicCapabilitySnapshot& Capability,
	FAircraftTrajectoryReference& OutReference)
{
	OutReference = {};
	if (!Plan.IsValid())
	{
		return false;
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
	FAircraftTrajectoryReference Candidate;
	const bool bSolved = Plan.GetIntent().Type == EAircraftMovementIntentType::Velocity
		? SolveVelocityIntent(State, Capability, Candidate)
		: SolvePlan(State, Capability, Candidate);
	const double ElapsedMilliseconds = (FPlatformTime::Seconds() - StartSeconds) * 1000.0;
	Diagnostics.LastSolveMilliseconds = ElapsedMilliseconds;
	Diagnostics.MaximumSolveMilliseconds = FMath::Max(
		Diagnostics.MaximumSolveMilliseconds, ElapsedMilliseconds);
	Diagnostics.SolverIterations = RuntimeConfig.Mpcc.MaxOptimizationIterations;
	if (!bSolved || ElapsedMilliseconds > RuntimeConfig.Mpcc.SolveTimeBudgetMilliseconds)
	{
		++Diagnostics.ConsecutiveFailures;
		Diagnostics.bReferenceFresh = false;
		Diagnostics.bSolverFailed = Diagnostics.ConsecutiveFailures
			>= RuntimeConfig.Mpcc.MaxConsecutiveFailures;
		if (Diagnostics.ConsecutiveFailures <= RuntimeConfig.Mpcc.MaxConsecutiveFailures
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

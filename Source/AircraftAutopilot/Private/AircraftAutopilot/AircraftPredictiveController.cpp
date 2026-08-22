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
	AccelerationHorizon.Reset();
	IntentRevision = 0;
	ActiveIntentId = 0;
	PlanRevision = 0;
	EstimatedPlanTimeSeconds = 0.0f;
	EstimatedDistanceCm = 0.0f;
	LastSolveTimeSeconds = -DBL_MAX;
}

bool FAircraftPredictiveController::SetIntent(
	const FAircraftMovementIntent& Intent, int64 InIntentId, uint64 InIntentRevision,
	const FAircraftAutopilotRuntimeConfig& Config,
	const FAircraftVehicleStateSnapshot& State,
	const FAircraftDynamicCapabilitySnapshot& Capability)
{
	RuntimeConfig = Config;
	IntentRevision = InIntentRevision;
	ActiveIntentId = InIntentId;
	Diagnostics = {};
	Diagnostics.ActiveIntentId = InIntentId;
	Diagnostics.IntentRevision = InIntentRevision;
	AccelerationHorizon.Reset();
	EstimatedPlanTimeSeconds = 0.0f;
	EstimatedDistanceCm = 0.0f;
	LastReference = {};
	LastSolveTimeSeconds = -DBL_MAX;
	const bool bBuilt = Plan.Build(Intent, Config, State, Capability);
	if (bBuilt)
	{
		++PlanRevision;
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
	float DesiredYawDegrees, float DesiredYawRateDegPerSec,
	FAircraftTrajectoryReference& InOutReference) const
{
	const float CurrentYaw = State.BodyRotation.Rotator().Yaw;
	const float TrackingAlpha = 1.0f - FMath::Exp(-FMath::Max(
		0.0f, RuntimeConfig.Mpcc.YawTrackingWeight));
	InOutReference.YawDegrees = CurrentYaw + FMath::FindDeltaAngleDegrees(
		CurrentYaw, DesiredYawDegrees) * TrackingAlpha;
	const float CurrentYawRate = FMath::RadiansToDegrees(
		static_cast<float>(State.AngularVelocityBodyRadPerSec.Z));
	const float RequestedRate = FMath::Clamp(DesiredYawRateDegPerSec,
		-Limits.MaxYawRateDegPerSec, Limits.MaxYawRateDegPerSec);
	float YawAcceleration = (RequestedRate - CurrentYawRate) / FMath::Max(DeltaTime, UE_SMALL_NUMBER);
	YawAcceleration = FMath::Clamp(YawAcceleration,
		-Limits.MaxYawAccelerationDegPerSecSq, Limits.MaxYawAccelerationDegPerSecSq);
	if (LastReference.bValid)
	{
		const float MaximumAccelerationStep = Limits.MaxYawJerkDegPerSecCubed * DeltaTime;
		YawAcceleration = FMath::Clamp(YawAcceleration,
			LastReference.YawAccelerationDegPerSecSq - MaximumAccelerationStep,
			LastReference.YawAccelerationDegPerSecSq + MaximumAccelerationStep);
	}
	InOutReference.YawAccelerationDegPerSecSq = YawAcceleration;
	InOutReference.YawRateDegPerSec = FMath::Clamp(
		CurrentYawRate + YawAcceleration * DeltaTime,
		-Limits.MaxYawRateDegPerSec, Limits.MaxYawRateDegPerSec);
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

	const float Dt = 1.0f / RuntimeConfig.Mpcc.UpdateRateHz;
	FVector Acceleration = ProjectAcceleration((TargetVelocity - State.VelocityCmPerSec) / Dt,
		Intent.Limits, Capability, State.VelocityCmPerSec);
	Acceleration = ApplyJerkLimit(
		LastReference.bValid ? LastReference.AccelerationCmPerSecSq
			: State.AccelerationCmPerSecSq,
		Acceleration, Dt, Intent.Limits);
	const FVector DragCompensation = ComputeDragCompensation(
		TargetVelocity, State.BodyRotation, Capability);
	Acceleration = ProjectAcceleration(Acceleration + DragCompensation,
		Intent.Limits, Capability, State.VelocityCmPerSec);

	OutReference.PositionCm = State.PositionCm + TargetVelocity * Dt;
	OutReference.VelocityCmPerSec = TargetVelocity;
	OutReference.AccelerationCmPerSecSq = Acceleration;
	const float DesiredYaw = FAircraftMotionPlan::ResolveYaw(Intent.Heading,
		State.PositionCm, TargetVelocity, State.BodyRotation.Rotator().Yaw);
	const float DesiredYawRate = FMath::FindDeltaAngleDegrees(
		State.BodyRotation.Rotator().Yaw, DesiredYaw) / Dt;
	ApplyYawConstraints(State, Intent.Limits, Dt, DesiredYaw, DesiredYawRate, OutReference);
	OutReference.PathProgress = 0.0f;
	return true;
}

bool FAircraftPredictiveController::SolvePlan(
	const FAircraftVehicleStateSnapshot& State,
	const FAircraftDynamicCapabilitySnapshot& Capability,
	FAircraftTrajectoryReference& OutReference)
{
	FAircraftMotionPlanSample Projection;
	if (!Plan.Project(State.PositionCm, EstimatedDistanceCm, Projection))
	{
		return false;
	}
	EstimatedDistanceCm = Projection.DistanceCm;
	EstimatedPlanTimeSeconds = FMath::Max(EstimatedPlanTimeSeconds, Projection.TimeSeconds);

	const int32 Steps = FMath::Clamp(RuntimeConfig.Mpcc.HorizonSteps, 2, 64);
	const float Dt = RuntimeConfig.Mpcc.HorizonSeconds / static_cast<float>(Steps);
	TArray<FAircraftMotionPlanSample> References;
	References.SetNum(Steps + 1);
	for (int32 Index = 0; Index <= Steps; ++Index)
	{
		if (!Plan.Evaluate(EstimatedPlanTimeSeconds + Dt * Index, References[Index]))
		{
			return false;
		}
	}

	const bool bInitializeHorizon = AccelerationHorizon.Num() != Steps;
	AccelerationHorizon.SetNum(Steps);
	for (int32 Index = 0; Index < Steps; ++Index)
	{
		if (bInitializeHorizon || AccelerationHorizon[Index].ContainsNaN())
		{
			AccelerationHorizon[Index] = References[Index].AccelerationCmPerSecSq;
		}
	}
	TArray<FVector> Positions;
	TArray<FVector> Velocities;
	Positions.SetNum(Steps + 1);
	Velocities.SetNum(Steps + 1);
	TArray<FVector> Gradient;
	Gradient.SetNum(Steps);

	const FAircraftMpccRuntimeConfig& Mpcc = RuntimeConfig.Mpcc;
	const FAircraftRequestedMotionLimits& Limits = Plan.GetIntent().Limits;
	for (int32 Iteration = 0; Iteration < Mpcc.MaxOptimizationIterations; ++Iteration)
	{
		Positions[0] = State.PositionCm;
		Velocities[0] = State.VelocityCmPerSec;
		for (int32 Index = 0; Index < Steps; ++Index)
		{
			AccelerationHorizon[Index] = ProjectAcceleration(
				AccelerationHorizon[Index], Limits, Capability, Velocities[Index]);
			AccelerationHorizon[Index] = ApplyJerkLimit(
				Index > 0 ? AccelerationHorizon[Index - 1] : State.AccelerationCmPerSecSq,
				AccelerationHorizon[Index], Dt, Limits);
			Positions[Index + 1] = Positions[Index] + Velocities[Index] * Dt
				+ 0.5f * AccelerationHorizon[Index] * Dt * Dt;
			Velocities[Index + 1] = Velocities[Index] + AccelerationHorizon[Index] * Dt;
		}

		FVector LambdaPosition = FVector::ZeroVector;
		FVector LambdaVelocity = FVector::ZeroVector;
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
			LambdaVelocity += Dt * LambdaPosition
				+ 2.0f * (Mpcc.SpeedTrackingWeight + TerminalVelocityWeight) * VelocityError;
			Gradient[Index] = 0.5f * Dt * Dt * LambdaPosition + Dt * LambdaVelocity
				+ 2.0f * (Mpcc.AccelerationWeight + Mpcc.Regularization) * AccelerationHorizon[Index];
			if (Index > 0)
			{
				Gradient[Index] += 2.0f * Mpcc.JerkWeight
					* (AccelerationHorizon[Index] - AccelerationHorizon[Index - 1]);
			}
		}

		const float StepSize = 1.0f / FMath::Max(1.0f,
			2.0f * (Mpcc.ContourErrorWeight + Mpcc.LagErrorWeight
				+ Mpcc.SpeedTrackingWeight) * Steps);
		for (int32 Index = 0; Index < Steps; ++Index)
		{
			AccelerationHorizon[Index] -= Gradient[Index] * StepSize;
		}
	}

	OutReference.PositionCm = References[0].PositionCm;
	OutReference.VelocityCmPerSec = References[0].VelocityCmPerSec;
	const FVector DragCompensation = ComputeDragCompensation(
		References[0].VelocityCmPerSec, State.BodyRotation, Capability);
	OutReference.AccelerationCmPerSecSq = ProjectAcceleration(
		AccelerationHorizon[0] + DragCompensation, Limits, Capability,
		State.VelocityCmPerSec);
	ApplyYawConstraints(State, Limits, 1.0f / Mpcc.UpdateRateHz,
		References[0].YawDegrees, References[0].YawRateDegPerSec, OutReference);
	OutReference.PathProgress = Plan.GetLengthCm() > UE_SMALL_NUMBER
		? FMath::Clamp(EstimatedDistanceCm / Plan.GetLengthCm(), 0.0f, 1.0f) : 0.0f;
	Diagnostics.ContourErrorCm = static_cast<float>((State.PositionCm - Projection.PositionCm
		- Projection.VelocityCmPerSec.GetSafeNormal()
		* FVector::DotProduct(State.PositionCm - Projection.PositionCm,
			Projection.VelocityCmPerSec.GetSafeNormal())).Size());
	Diagnostics.LagErrorCm = static_cast<float>(FVector::DotProduct(
		State.PositionCm - Projection.PositionCm, Projection.VelocityCmPerSec.GetSafeNormal()));
	EstimatedPlanTimeSeconds += FMath::Clamp(Mpcc.ProgressWeight, 0.0f, 4.0f)
		/ Mpcc.UpdateRateHz;
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
		&& State.TimeSeconds - LastSolveTimeSeconds < SolveInterval)
	{
		OutReference = LastReference;
		Diagnostics.bReferenceFresh = true;
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
		if (Diagnostics.ConsecutiveFailures <= RuntimeConfig.Mpcc.MaxConsecutiveFailures
			&& LastReference.IsFresh(State.TimeSeconds))
		{
			OutReference = LastReference;
			Diagnostics.bReferenceFresh = true;
			return true;
		}
		return false;
	}

	Diagnostics.ConsecutiveFailures = 0;
	Candidate.IntentId = ActiveIntentId;
	Candidate.IntentRevision = IntentRevision;
	Candidate.PlanRevision = PlanRevision;
	Candidate.StateSequence = State.Sequence;
	Candidate.GeneratedAtSeconds = State.TimeSeconds;
	Candidate.ValidUntilSeconds = State.TimeSeconds + RuntimeConfig.Mpcc.MaximumReferenceAgeSeconds;
	Candidate.YawRateLimitDegPerSec = Plan.GetIntent().Limits.MaxYawRateDegPerSec;
	Candidate.bValid = true;
	LastSolveTimeSeconds = State.TimeSeconds;
	LastReference = Candidate;
	OutReference = Candidate;
	Diagnostics.PlanRevision = PlanRevision;
	Diagnostics.bReferenceFresh = true;
	return true;
}

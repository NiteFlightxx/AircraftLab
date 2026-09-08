#include "AircraftAutopilot/AircraftTrajectoryRuntime.h"

#include "ProfilingDebugging/CpuProfilerTrace.h"

namespace
{
	float HardLimit(float Requested, float Available)
	{
		if (Requested <= 0.0f) return Available;
		if (Available <= 0.0f) return Requested;
		return FMath::Min(Requested, Available);
	}

	FVector ClampAcceleration(const FVector& Desired, const FVector& Velocity,
		const FAircraftRequestedMotionLimits& Limits,
		const FAircraftDynamicCapabilitySnapshot& Capability)
	{
		FVector Result = Desired;
		const FVector2D Horizontal(Result.X, Result.Y);
		const bool bBraking = FVector2D::DotProduct(Horizontal,
			FVector2D(Velocity.X, Velocity.Y)) < 0.0f;
		const float HorizontalLimit = HardLimit(
			bBraking ? Limits.MaxDecelerationCmPerSecSq : Limits.MaxAccelerationCmPerSecSq,
			Capability.MaxHorizontalAccelerationCmPerSecSq);
		if (HorizontalLimit > 0.0f && Horizontal.SizeSquared() > FMath::Square(HorizontalLimit))
		{
			const FVector2D Limited = Horizontal.GetSafeNormal() * HorizontalLimit;
			Result.X = Limited.X;
			Result.Y = Limited.Y;
		}
		const float VerticalLimit = HardLimit(
			Limits.MaxVerticalAccelerationCmPerSecSq,
			Capability.MaxVerticalAccelerationCmPerSecSq);
		if (VerticalLimit > 0.0f)
		{
			Result.Z = FMath::Clamp(Result.Z, -VerticalLimit, VerticalLimit);
		}
		return Result;
	}

	FVector ApplyJerk(const FVector& Previous, const FVector& Desired, float DeltaTime,
		const FAircraftRequestedMotionLimits& Limits)
	{
		FVector Delta = Desired - Previous;
		const float HorizontalStep = FMath::Max(Limits.MaxJerkCmPerSecCubed, 0.0f) * DeltaTime;
		const FVector2D Horizontal = FVector2D(Delta.X, Delta.Y).GetClampedToMaxSize(HorizontalStep);
		const float VerticalStep = FMath::Max(Limits.MaxVerticalJerkCmPerSecCubed, 0.0f) * DeltaTime;
		Delta = FVector(Horizontal.X, Horizontal.Y,
			FMath::Clamp(Delta.Z, -VerticalStep, VerticalStep));
		return Previous + Delta;
	}

	FVector ComputeDynamicsFeedForward(const FVector& VelocityWorldCmPerSec,
		const FQuat& BodyRotation, const FAircraftDynamicCapabilitySnapshot& Capability)
	{
		FVector Result = Capability.LinearDampingPerSecond * VelocityWorldCmPerSec;
		if (!Capability.bHasExplicitAerodynamics || Capability.MassKg <= UE_SMALL_NUMBER)
		{
			return Result;
		}
		const FVector VelocityBodyMps = BodyRotation.UnrotateVector(
			VelocityWorldCmPerSec) * 0.01f;
		FVector VelocityAircraftMps = Capability.AircraftToBodyRotation.UnrotateVector(
			VelocityBodyMps);
		VelocityAircraftMps = VelocityAircraftMps.GetClampedToMaxSize(
			Capability.MaxRelativeAirspeedCmPerSec * 0.01f);
		const FVector Quadratic = Capability.DragAreaCoefficientAircraftM2
			* (0.5f * Capability.AirDensityKgPerM3);
		const FVector ForceAircraftN =
			Capability.LinearDragAircraftNsPerM * VelocityAircraftMps
			+ Quadratic * VelocityAircraftMps.GetAbs() * VelocityAircraftMps;
		const FVector ForceBodyN = Capability.AircraftToBodyRotation.RotateVector(
			ForceAircraftN);
		return Result + BodyRotation.RotateVector(ForceBodyN)
			* (100.0f / Capability.MassKg);
	}

	void ClampGuidanceSample(FAircraftNavigationGuidanceSample& Sample,
		const FAircraftDynamicCapabilitySnapshot& Capability)
	{
		const FVector2D HorizontalVelocity(Sample.VelocityCmPerSec.X, Sample.VelocityCmPerSec.Y);
		if (Capability.MaxHorizontalSpeedCmPerSec > 0.0f
			&& HorizontalVelocity.SizeSquared() > FMath::Square(Capability.MaxHorizontalSpeedCmPerSec))
		{
			const FVector2D Limited = HorizontalVelocity.GetSafeNormal()
				* Capability.MaxHorizontalSpeedCmPerSec;
			Sample.VelocityCmPerSec.X = Limited.X;
			Sample.VelocityCmPerSec.Y = Limited.Y;
		}
		Sample.VelocityCmPerSec.Z = FMath::Clamp(
			Sample.VelocityCmPerSec.Z,
			-FMath::Max(Capability.MaxDescentRateCmPerSec, 0.0f),
			FMath::Max(Capability.MaxClimbRateCmPerSec, 0.0f));

		FVector2D HorizontalAcceleration(
			Sample.AccelerationCmPerSecSq.X, Sample.AccelerationCmPerSecSq.Y);
		const bool bBraking = FVector2D::DotProduct(HorizontalAcceleration,
			FVector2D(Sample.VelocityCmPerSec.X, Sample.VelocityCmPerSec.Y)) < 0.0f;
		const float HorizontalAccelerationLimit = bBraking
			? Capability.MaxHorizontalDecelerationCmPerSecSq
			: Capability.MaxHorizontalAccelerationCmPerSecSq;
		if (HorizontalAccelerationLimit > 0.0f)
		{
			HorizontalAcceleration = HorizontalAcceleration.GetClampedToMaxSize(
				HorizontalAccelerationLimit);
			Sample.AccelerationCmPerSecSq.X = HorizontalAcceleration.X;
			Sample.AccelerationCmPerSecSq.Y = HorizontalAcceleration.Y;
		}
		if (Capability.MaxVerticalAccelerationCmPerSecSq > 0.0f)
		{
			Sample.AccelerationCmPerSecSq.Z = FMath::Clamp(
				Sample.AccelerationCmPerSecSq.Z,
				-Capability.MaxVerticalAccelerationCmPerSecSq,
				Capability.MaxVerticalAccelerationCmPerSecSq);
		}
	}
}

bool FAircraftTrajectoryRuntime::SetIntent(const FAircraftMovementIntent& InIntent,
	int64 IntentId, uint64 IntentRevision,
	const FAircraftAutopilotRuntimeConfig& Config,
	const FAircraftVehicleStateSnapshot& State,
	const FAircraftDynamicCapabilitySnapshot& Capability)
{
	TRACE_CPUPROFILER_EVENT_SCOPE(Aircraft_Trajectory_SetIntent);
	const FAircraftAutopilotDiagnostics PreviousDiagnostics = MpccController.GetDiagnostics();
	const bool bSameIntent = PreviousDiagnostics.ActiveIntentId == IntentId
		&& GetPlan().IsValid() && GetPlan().GetIntent().Type == InIntent.Type;
	GovernorScaleCm = FMath::Max(Config.Tracking.ContourErrorGovernorScaleCm, 1.0f);
	GovernorResponseRatePerSecond = FMath::Max(
		Config.Tracking.ProgressScaleResponseRatePerSecond, UE_SMALL_NUMBER);
	const bool bBuilt = MpccController.SetIntent(
		InIntent, IntentId, IntentRevision, Config, State, Capability);
	Diagnostics = MpccController.GetDiagnostics();
	if (!bBuilt)
	{
		return false;
	}
	if (!bSameIntent)
	{
		PlanTimeSeconds = 0.0f;
		PlanDistanceCm = 0.0f;
		ProgressScale = 1.0f;
		PositionReferenceCm = State.PositionCm;
		VelocityReferenceCmPerSec = State.VelocityCmPerSec;
		VelocityAccelerationCmPerSecSq = FVector::ZeroVector;
		LastReference = {};
	}
	FAircraftMotionPlanSample Projection;
	if (GetPlan().Project(State.PositionCm, PlanDistanceCm, !bSameIntent, Projection))
	{
		PlanDistanceCm = Projection.DistanceCm;
		PlanTimeSeconds = GetPlan().TimeAtDistance(PlanDistanceCm);
	}
	LastUpdateTimeSeconds = State.TimeSeconds;
	return true;
}

void FAircraftTrajectoryRuntime::Reset()
{
	MpccController.Reset();
	Diagnostics = {};
	LastReference = {};
	LastNominalReference = {};
	VelocityReferenceCmPerSec = FVector::ZeroVector;
	VelocityAccelerationCmPerSecSq = FVector::ZeroVector;
	PositionReferenceCm = FVector::ZeroVector;
	PlanTimeSeconds = 0.0f;
	PlanDistanceCm = 0.0f;
	ProgressScale = 1.0f;
	LastUpdateTimeSeconds = 0.0;
}

void FAircraftTrajectoryRuntime::RebaseTime(const double TimeSeconds)
{
	MpccController.RebaseTime(TimeSeconds);
	LastUpdateTimeSeconds = TimeSeconds;
}

bool FAircraftTrajectoryRuntime::SetNavigationGuidance(
	TSharedPtr<const FAircraftNavigationGuidance, ESPMode::ThreadSafe> Guidance,
	const uint64 Revision)
{
	NavigationGuidanceStatus = {};
	NavigationGuidanceStatus.Revision = Revision;
	NavigationGuidanceStatus.GeneratedAtSeconds = Guidance.IsValid()
		? Guidance->GeneratedAtSeconds : 0.0;
	NavigationGuidanceStatus.ValidUntilSeconds = Guidance.IsValid()
		? Guidance->ValidUntilSeconds : 0.0;
	bNavigationGuidanceActive = true;
	bNavigationGuidanceAvailable = Guidance.IsValid() && Guidance->IsValid();
	if (!bNavigationGuidanceAvailable)
	{
		NavigationGuidance.Reset();
		NavigationGuidanceStatus.State = EAircraftNavigationGuidanceState::Braking;
		NavigationGuidanceStatus.FailureReason =
			EAircraftNavigationGuidanceFailureReason::InvalidGuidance;
		return false;
	}
	NavigationGuidance = MoveTemp(Guidance);
	NavigationGuidanceStatus.State = NavigationGuidance->Mode == EAircraftNavigationGuidanceMode::Brake
		? EAircraftNavigationGuidanceState::Braking
		: EAircraftNavigationGuidanceState::Applied;
	return true;
}

void FAircraftTrajectoryRuntime::SetNavigationGuidanceUnavailable(const uint64 Revision)
{
	NavigationGuidance.Reset();
	NavigationGuidanceStatus = {};
	NavigationGuidanceStatus.Revision = Revision;
	NavigationGuidanceStatus.State = EAircraftNavigationGuidanceState::Braking;
	NavigationGuidanceStatus.FailureReason =
		EAircraftNavigationGuidanceFailureReason::Unavailable;
	bNavigationGuidanceActive = true;
	bNavigationGuidanceAvailable = false;
}

void FAircraftTrajectoryRuntime::ClearNavigationGuidance(const uint64 Revision)
{
	NavigationGuidance.Reset();
	NavigationGuidanceStatus = {};
	NavigationGuidanceStatus.Revision = Revision;
	bNavigationGuidanceActive = false;
	bNavigationGuidanceAvailable = false;
}

bool FAircraftTrajectoryRuntime::UpdateFlightController(
	const FAircraftVehicleStateSnapshot& State,
	const FAircraftDynamicCapabilitySnapshot& Capability,
	FAircraftTrajectoryReference& OutReference)
{
	TRACE_CPUPROFILER_EVENT_SCOPE(Aircraft_MPCC_Update);
	const float GuidanceDeltaTime = FMath::Clamp(
		static_cast<float>(State.TimeSeconds - LastUpdateTimeSeconds), 0.0f, 0.1f);
	const bool bUpdated = MpccController.Update(State, Capability, OutReference);
	Diagnostics = MpccController.GetDiagnostics();
	if (bUpdated)
	{
		PlanTimeSeconds = GetPlan().TimeAtDistance(
			OutReference.PathProgress * GetPlan().GetLengthCm());
		PlanDistanceCm = OutReference.PathProgress * GetPlan().GetLengthCm();
		LastUpdateTimeSeconds = State.TimeSeconds;
		LastNominalReference = OutReference;
		ApplyNavigationGuidance(State, Capability, GuidanceDeltaTime, false, OutReference);
		LastReference = OutReference;
	}
	return bUpdated;
}

bool FAircraftTrajectoryRuntime::UpdatePhysicsConstraint(
	const FAircraftVehicleStateSnapshot& State,
	const FAircraftDynamicCapabilitySnapshot& Capability,
	FAircraftTrajectoryReference& OutReference)
{
	TRACE_CPUPROFILER_EVENT_SCOPE(Aircraft_Constraint_TrajectoryUpdate);
	return UpdateDeterministic(State, Capability, true, true, OutReference);
}

bool FAircraftTrajectoryRuntime::UpdateKinematic(
	const FAircraftVehicleStateSnapshot& State,
	const FAircraftDynamicCapabilitySnapshot& Capability,
	FAircraftTrajectoryReference& OutReference)
{
	TRACE_CPUPROFILER_EVENT_SCOPE(Aircraft_Kinematic_TrajectoryUpdate);
	return UpdateDeterministic(State, Capability, false, false, OutReference);
}

bool FAircraftTrajectoryRuntime::UpdateVelocity(
	const FAircraftVehicleStateSnapshot& State,
	const FAircraftDynamicCapabilitySnapshot& Capability,
	bool bUseDynamics, float DeltaTime,
	FAircraftTrajectoryReference& OutReference)
{
	const FAircraftMovementIntent& Intent = GetPlan().GetIntent();
	FVector TargetVelocity = Intent.Velocity.VelocityCmPerSec;
	if (Intent.Velocity.Frame == EAircraftVelocityFrame::ControlHeading)
	{
		TargetVelocity = State.ControlRotation.RotateVector(TargetVelocity);
	}
	const float HorizontalSpeed = FVector2D(TargetVelocity.X, TargetVelocity.Y).Size();
	const float SpeedLimit = HardLimit(
		Intent.Limits.CruiseSpeedCmPerSec, Capability.MaxHorizontalSpeedCmPerSec);
	if (SpeedLimit > 0.0f && HorizontalSpeed > SpeedLimit)
	{
		const FVector2D Limited = FVector2D(TargetVelocity.X, TargetVelocity.Y)
			.GetSafeNormal() * SpeedLimit;
		TargetVelocity.X = Limited.X;
		TargetVelocity.Y = Limited.Y;
	}
	TargetVelocity.Z = FMath::Clamp(TargetVelocity.Z,
		-HardLimit(Intent.Limits.MaxDescentRateCmPerSec, Capability.MaxDescentRateCmPerSec),
		HardLimit(Intent.Limits.MaxClimbRateCmPerSec, Capability.MaxClimbRateCmPerSec));

	const FVector DesiredAcceleration = ClampAcceleration(
		(TargetVelocity - VelocityReferenceCmPerSec) / FMath::Max(DeltaTime, UE_SMALL_NUMBER),
		VelocityReferenceCmPerSec, Intent.Limits, Capability);
	VelocityAccelerationCmPerSecSq = ApplyJerk(
		VelocityAccelerationCmPerSecSq, DesiredAcceleration, DeltaTime, Intent.Limits);
	const FVector PreviousVelocity = VelocityReferenceCmPerSec;
	VelocityReferenceCmPerSec += VelocityAccelerationCmPerSecSq * DeltaTime;
	if (FVector::DotProduct(TargetVelocity - PreviousVelocity,
		TargetVelocity - VelocityReferenceCmPerSec) <= 0.0f)
	{
		VelocityReferenceCmPerSec = TargetVelocity;
		VelocityAccelerationCmPerSecSq = FVector::ZeroVector;
	}
	PositionReferenceCm += 0.5f * (PreviousVelocity + VelocityReferenceCmPerSec) * DeltaTime;
	OutReference.PositionCm = PositionReferenceCm;
	OutReference.VelocityCmPerSec = VelocityReferenceCmPerSec;
	OutReference.AccelerationCmPerSecSq = VelocityAccelerationCmPerSecSq;
	OutReference.ControlAccelerationCmPerSecSq = VelocityAccelerationCmPerSecSq;
	OutReference.DynamicsFeedForwardAccelerationCmPerSecSq = bUseDynamics
		? ComputeDynamicsFeedForward(VelocityReferenceCmPerSec, State.BodyRotation, Capability)
		: FVector::ZeroVector;
	OutReference.YawDegrees = FAircraftMotionPlan::ResolveYaw(
		Intent.Heading, PositionReferenceCm, VelocityReferenceCmPerSec,
		LastReference.bValid ? LastReference.YawDegrees : State.ControlRotation.Rotator().Yaw);
	OutReference.YawRateLimitDegPerSec = Intent.Limits.MaxYawRateDegPerSec;
	OutReference.bPositionTrackingEnabled = false;
	return true;
}

bool FAircraftTrajectoryRuntime::UpdateDeterministic(
	const FAircraftVehicleStateSnapshot& State,
	const FAircraftDynamicCapabilitySnapshot& Capability,
	bool bUseProgressGovernor, bool bUseDynamics,
	FAircraftTrajectoryReference& OutReference)
{
	if (!MpccController.RefreshPlan(State, Capability) || !GetPlan().IsValid())
	{
		Diagnostics.bPlanValid = false;
		return false;
	}
	const FAircraftMovementIntent& Intent = GetPlan().GetIntent();
	const float DeltaTime = FMath::Clamp(
		static_cast<float>(State.TimeSeconds - LastUpdateTimeSeconds), 0.0f, 0.1f);
	LastUpdateTimeSeconds = State.TimeSeconds;
	OutReference = {};
	if (Intent.Type == EAircraftMovementIntentType::Velocity)
	{
		if (!UpdateVelocity(State, Capability, bUseDynamics, DeltaTime, OutReference))
		{
			return false;
		}
	}
	else
	{
		if (bUseProgressGovernor)
		{
			FAircraftMotionPlanSample Projection;
			if (GetPlan().Project(State.PositionCm, PlanDistanceCm, false, Projection))
			{
				PlanDistanceCm = FMath::Max(PlanDistanceCm, Projection.DistanceCm);
				PlanTimeSeconds = FMath::Max(PlanTimeSeconds,
					GetPlan().TimeAtDistance(PlanDistanceCm));
				const float ContourError = FVector::Distance(State.PositionCm, Projection.PositionCm);
				const float NormalizedError = ContourError / GovernorScaleCm;
				const float TargetScale = 1.0f / (1.0f + NormalizedError * NormalizedError);
				ProgressScale = FMath::FInterpTo(
					ProgressScale, TargetScale, DeltaTime, GovernorResponseRatePerSecond);
				Diagnostics.ContourErrorCm = ContourError;
				Diagnostics.CorridorViolationCm = GetPlan().ComputeCorridorViolationCm(
					State.PositionCm, PlanDistanceCm);
			}
		}
		else
		{
			ProgressScale = 1.0f;
		}
		PlanTimeSeconds += DeltaTime * ProgressScale;
		FAircraftMotionPlanSample Sample;
		if (!GetPlan().Evaluate(PlanTimeSeconds, Sample))
		{
			return false;
		}
		PlanDistanceCm = Sample.DistanceCm;
		OutReference.PositionCm = Sample.PositionCm;
		OutReference.VelocityCmPerSec = Sample.VelocityCmPerSec;
		OutReference.AccelerationCmPerSecSq = Sample.AccelerationCmPerSecSq;
		OutReference.ControlAccelerationCmPerSecSq = Sample.AccelerationCmPerSecSq;
		OutReference.DynamicsFeedForwardAccelerationCmPerSecSq = bUseDynamics
			? ComputeDynamicsFeedForward(Sample.VelocityCmPerSec, State.BodyRotation, Capability)
			: FVector::ZeroVector;
		OutReference.YawDegrees = Sample.YawDegrees;
		OutReference.YawRateDegPerSec = Sample.YawRateDegPerSec;
		OutReference.YawRateLimitDegPerSec = Intent.Limits.MaxYawRateDegPerSec;
		OutReference.bPositionTrackingEnabled = true;
	}
	FinalizeReference(State, OutReference);
	LastNominalReference = OutReference;
	ApplyNavigationGuidance(State, Capability, DeltaTime, !bUseDynamics, OutReference);
	LastReference = OutReference;
	return true;
}

void FAircraftTrajectoryRuntime::FinalizeReference(
	const FAircraftVehicleStateSnapshot& State,
	FAircraftTrajectoryReference& OutReference)
{
	const FAircraftAutopilotDiagnostics& MpccDiagnostics = MpccController.GetDiagnostics();
	OutReference.IntentId = MpccDiagnostics.ActiveIntentId;
	OutReference.IntentRevision = MpccDiagnostics.IntentRevision;
	OutReference.PlanRevision = MpccDiagnostics.PlanRevision;
	OutReference.StateSequence = State.Sequence;
	OutReference.GeneratedAtSeconds = State.TimeSeconds;
	OutReference.ValidUntilSeconds = State.TimeSeconds + 0.15;
	OutReference.PathProgress = GetPlan().GetLengthCm() > UE_SMALL_NUMBER
		? PlanDistanceCm / GetPlan().GetLengthCm() : 0.0f;
	OutReference.RouteProgress = GetPlan().GetRouteLengthCm() > UE_SMALL_NUMBER
		? GetPlan().GetRouteDistanceCm(PlanDistanceCm) / GetPlan().GetRouteLengthCm()
		: 0.0f;
	OutReference.bValid = true;
	Diagnostics = MpccDiagnostics;
	Diagnostics.ProgressScale = ProgressScale;
	Diagnostics.bReferenceFresh = true;
}

void FAircraftTrajectoryRuntime::ApplyNavigationGuidance(
	const FAircraftVehicleStateSnapshot& State,
	const FAircraftDynamicCapabilitySnapshot& Capability,
	const float DeltaTime, const bool bKinematic,
	FAircraftTrajectoryReference& InOutReference)
{
	if (!bNavigationGuidanceActive)
	{
		return;
	}
	if (!bNavigationGuidanceAvailable)
	{
		BuildBrakingReference(State, Capability,
			NavigationGuidanceStatus.FailureReason, DeltaTime, bKinematic, InOutReference);
		return;
	}
	if (NavigationGuidance->SourceIntentId != InOutReference.IntentId
		|| NavigationGuidance->SourceIntentRevision
			!= static_cast<uint64>(InOutReference.IntentRevision))
	{
		NavigationGuidanceStatus.State = EAircraftNavigationGuidanceState::Inactive;
		NavigationGuidanceStatus.FailureReason =
			EAircraftNavigationGuidanceFailureReason::IntentMismatch;
		return;
	}
	if (!NavigationGuidance->IsFresh(State.TimeSeconds))
	{
		BuildBrakingReference(State, Capability,
			EAircraftNavigationGuidanceFailureReason::Expired,
			DeltaTime, bKinematic, InOutReference);
		return;
	}
	if (NavigationGuidance->Mode == EAircraftNavigationGuidanceMode::Brake)
	{
		BuildBrakingReference(State, Capability,
			EAircraftNavigationGuidanceFailureReason::None,
			DeltaTime, bKinematic, InOutReference);
		return;
	}

	FAircraftNavigationGuidanceSample Sample;
	if (!NavigationGuidance->Evaluate(State.TimeSeconds, Sample))
	{
		BuildBrakingReference(State, Capability,
			EAircraftNavigationGuidanceFailureReason::InvalidGuidance,
			DeltaTime, bKinematic, InOutReference);
		return;
	}
	ClampGuidanceSample(Sample, Capability);
	InOutReference.PositionCm = Sample.PositionCm;
	InOutReference.VelocityCmPerSec = Sample.VelocityCmPerSec;
	InOutReference.AccelerationCmPerSecSq = Sample.AccelerationCmPerSecSq;
	InOutReference.ControlAccelerationCmPerSecSq = Sample.AccelerationCmPerSecSq;
	InOutReference.DynamicsFeedForwardAccelerationCmPerSecSq =
		ComputeDynamicsFeedForward(Sample.VelocityCmPerSec, State.BodyRotation, Capability);
	InOutReference.bPositionTrackingEnabled = true;
	InOutReference.ValidUntilSeconds = FMath::Min(
		InOutReference.ValidUntilSeconds, NavigationGuidance->ValidUntilSeconds);
	NavigationGuidanceStatus.State = EAircraftNavigationGuidanceState::Applied;
	NavigationGuidanceStatus.FailureReason = EAircraftNavigationGuidanceFailureReason::None;
}

void FAircraftTrajectoryRuntime::BuildBrakingReference(
	const FAircraftVehicleStateSnapshot& State,
	const FAircraftDynamicCapabilitySnapshot& Capability,
	const EAircraftNavigationGuidanceFailureReason FailureReason,
	const float DeltaTime, const bool bKinematic,
	FAircraftTrajectoryReference& InOutReference)
{
	const FVector2D HorizontalVelocity(State.VelocityCmPerSec.X, State.VelocityCmPerSec.Y);
	const float HorizontalDeceleration = FMath::Max(
		Capability.MaxHorizontalDecelerationCmPerSecSq, 1.0f);
	const FVector2D HorizontalOffset = HorizontalVelocity
		* (HorizontalVelocity.Size() / (2.0f * HorizontalDeceleration));
	const FVector2D HorizontalAcceleration = HorizontalVelocity.IsNearlyZero()
		? FVector2D::ZeroVector
		: -HorizontalVelocity.GetSafeNormal() * HorizontalDeceleration;
	const float VerticalAcceleration = FMath::Max(
		Capability.MaxVerticalAccelerationCmPerSecSq, 1.0f);
	const float VerticalOffset = State.VelocityCmPerSec.Z
		* FMath::Abs(State.VelocityCmPerSec.Z) / (2.0f * VerticalAcceleration);
	const float VerticalBrakingAcceleration = FMath::IsNearlyZero(State.VelocityCmPerSec.Z)
		? 0.0f
		: -FMath::Sign(State.VelocityCmPerSec.Z) * VerticalAcceleration;

	const FVector BrakingAcceleration(
		HorizontalAcceleration.X, HorizontalAcceleration.Y, VerticalBrakingAcceleration);
	if (bKinematic)
	{
		const float StepSeconds = FMath::Clamp(DeltaTime, 0.0f, 0.1f);
		FVector NextVelocity = State.VelocityCmPerSec + BrakingAcceleration * StepSeconds;
		for (int32 Axis = 0; Axis < 3; ++Axis)
		{
			if (State.VelocityCmPerSec[Axis] * NextVelocity[Axis] <= 0.0f)
			{
				NextVelocity[Axis] = 0.0f;
			}
		}
		InOutReference.PositionCm = State.PositionCm
			+ 0.5f * (State.VelocityCmPerSec + NextVelocity) * StepSeconds;
		InOutReference.VelocityCmPerSec = NextVelocity;
	}
	else
	{
		InOutReference.PositionCm = State.PositionCm
			+ FVector(HorizontalOffset.X, HorizontalOffset.Y, VerticalOffset);
		InOutReference.VelocityCmPerSec = FVector::ZeroVector;
	}
	InOutReference.AccelerationCmPerSecSq = FVector(
		BrakingAcceleration.X, BrakingAcceleration.Y, BrakingAcceleration.Z);
	InOutReference.ControlAccelerationCmPerSecSq = InOutReference.AccelerationCmPerSecSq;
	InOutReference.DynamicsFeedForwardAccelerationCmPerSecSq = FVector::ZeroVector;
	InOutReference.bPositionTrackingEnabled = true;
	InOutReference.GeneratedAtSeconds = State.TimeSeconds;
	InOutReference.ValidUntilSeconds = State.TimeSeconds + 0.15;
	NavigationGuidanceStatus.State = EAircraftNavigationGuidanceState::Braking;
	NavigationGuidanceStatus.FailureReason = FailureReason;
}

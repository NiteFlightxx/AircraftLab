#include "AircraftAutopilot/AircraftTrajectoryRuntime.h"

#include "Aircraft/AircraftAttitudeReference.h"
#include "Aircraft/AircraftYawReferenceDynamics.h"
#include "AircraftAutopilotDynamics.h"
#include "ProfilingDebugging/CpuProfilerTrace.h"

namespace
{
	// Valid guidance ramps in to avoid a discontinuity. Missing, stale or invalid
	// guidance is a hard-safety event and therefore transitions directly to braking.
	constexpr float GuidanceBlendRampSeconds = 0.2f;

	float HardLimit(float Requested, float Available)
	{
		if (Requested <= 0.0f) return Available;
		if (Available <= 0.0f) return Requested;
		return FMath::Min(Requested, Available);
	}

	float ResolveVelocityNaturalFrequency(
		const float ErrorMagnitude,
		const float MaximumAcceleration,
		const float MaximumJerk)
	{
		const float SafeError = FMath::Max(ErrorMagnitude, 1.e-3f);
		const float JerkFrequency = FMath::Sqrt(MaximumJerk / SafeError);
		const float AccelerationFrequency = 4.0f * MaximumAcceleration / SafeError;
		return FMath::Max(FMath::Min(JerkFrequency, AccelerationFrequency), UE_SMALL_NUMBER);
	}

	void UpdateVelocityReferenceStep(
		const FVector& TargetVelocity,
		const float StepSeconds,
		const FAircraftRequestedMotionLimits& Limits,
		const FAircraftDynamicCapabilitySnapshot& Capability,
		FVector& InOutVelocity,
		FVector& InOutAcceleration)
	{
		const FVector2D HorizontalTarget(TargetVelocity.X, TargetVelocity.Y);
		FVector2D HorizontalVelocity(InOutVelocity.X, InOutVelocity.Y);
		FVector2D HorizontalAcceleration(InOutAcceleration.X, InOutAcceleration.Y);
		const FVector2D HorizontalError = HorizontalTarget - HorizontalVelocity;
		const bool bHorizontalBraking = FVector2D::DotProduct(
			HorizontalError, HorizontalVelocity) < 0.0f;
		const float HorizontalAccelerationLimit = HardLimit(
			bHorizontalBraking
				? Limits.MaxDecelerationCmPerSecSq
				: Limits.MaxAccelerationCmPerSecSq,
			bHorizontalBraking
				? Capability.MaxHorizontalDecelerationCmPerSecSq
				: Capability.MaxHorizontalAccelerationCmPerSecSq);
		const float HorizontalJerkLimit = HardLimit(
			Limits.MaxJerkCmPerSecCubed,
			Capability.MaxHorizontalJerkCmPerSecCubed);
		if (HorizontalAccelerationLimit > UE_SMALL_NUMBER
			&& HorizontalJerkLimit > UE_SMALL_NUMBER)
		{
			if (HorizontalError.Size() <= 1.e-2f
				&& HorizontalAcceleration.Size()
					<= HorizontalJerkLimit * StepSeconds)
			{
				HorizontalVelocity = HorizontalTarget;
				HorizontalAcceleration = FVector2D::ZeroVector;
			}
			else
			{
				const float Omega = ResolveVelocityNaturalFrequency(
					HorizontalError.Size(), HorizontalAccelerationLimit,
					HorizontalJerkLimit);
				const FVector2D RequestedJerk = (HorizontalError * FMath::Square(Omega)
					- HorizontalAcceleration * (2.0f * Omega))
					.GetClampedToMaxSize(HorizontalJerkLimit);
				HorizontalAcceleration = (HorizontalAcceleration
					+ RequestedJerk * StepSeconds)
					.GetClampedToMaxSize(HorizontalAccelerationLimit);
				HorizontalVelocity += HorizontalAcceleration * StepSeconds;
			}
		}
		else
		{
			HorizontalVelocity = HorizontalTarget;
			HorizontalAcceleration = FVector2D::ZeroVector;
		}

		const float VerticalError = TargetVelocity.Z - InOutVelocity.Z;
		const float VerticalAccelerationLimit = HardLimit(
			Limits.MaxVerticalAccelerationCmPerSecSq,
			Capability.MaxVerticalAccelerationCmPerSecSq);
		const float VerticalJerkLimit = HardLimit(
			Limits.MaxVerticalJerkCmPerSecCubed,
			Capability.MaxVerticalJerkCmPerSecCubed);
		if (VerticalAccelerationLimit > UE_SMALL_NUMBER
			&& VerticalJerkLimit > UE_SMALL_NUMBER)
		{
			if (FMath::Abs(VerticalError) <= 1.e-2f
				&& FMath::Abs(InOutAcceleration.Z)
					<= VerticalJerkLimit * StepSeconds)
			{
				InOutVelocity.Z = TargetVelocity.Z;
				InOutAcceleration.Z = 0.0f;
			}
			else
			{
				const float Omega = ResolveVelocityNaturalFrequency(
					FMath::Abs(VerticalError), VerticalAccelerationLimit,
					VerticalJerkLimit);
				const float RequestedJerk = FMath::Clamp(
					FMath::Square(Omega) * VerticalError
						- 2.0f * Omega * InOutAcceleration.Z,
					-VerticalJerkLimit, VerticalJerkLimit);
				InOutAcceleration.Z = FMath::Clamp(
					InOutAcceleration.Z + RequestedJerk * StepSeconds,
					-VerticalAccelerationLimit, VerticalAccelerationLimit);
				InOutVelocity.Z += InOutAcceleration.Z * StepSeconds;
			}
		}
		else
		{
			InOutVelocity.Z = TargetVelocity.Z;
			InOutAcceleration.Z = 0.0f;
		}

		InOutVelocity.X = HorizontalVelocity.X;
		InOutVelocity.Y = HorizontalVelocity.Y;
		InOutAcceleration.X = HorizontalAcceleration.X;
		InOutAcceleration.Y = HorizontalAcceleration.Y;
	}

bool IsGuidanceSampleWithinCapability(const FAircraftNavigationGuidanceSample& Sample,
		const FAircraftDynamicCapabilitySnapshot& Capability)
	{
		const auto IsFiniteVector = [](const FVector& Value)
		{
			return FMath::IsFinite(Value.X) && FMath::IsFinite(Value.Y)
				&& FMath::IsFinite(Value.Z);
		};
		if (!IsFiniteVector(Sample.PositionCm) || !IsFiniteVector(Sample.VelocityCmPerSec)
			|| !IsFiniteVector(Sample.AccelerationCmPerSecSq))
		{
			return false;
		}
		const FVector2D HorizontalVelocity(Sample.VelocityCmPerSec.X, Sample.VelocityCmPerSec.Y);
		if (Capability.MaxHorizontalSpeedCmPerSec > 0.0f
			&& HorizontalVelocity.SizeSquared() > FMath::Square(Capability.MaxHorizontalSpeedCmPerSec))
		{
			return false;
		}
		if ((Capability.MaxClimbRateCmPerSec > 0.0f
				&& Sample.VelocityCmPerSec.Z > Capability.MaxClimbRateCmPerSec)
			|| (Capability.MaxDescentRateCmPerSec > 0.0f
				&& Sample.VelocityCmPerSec.Z < -Capability.MaxDescentRateCmPerSec))
		{
			return false;
		}

		const FVector2D HorizontalAcceleration(
			Sample.AccelerationCmPerSecSq.X, Sample.AccelerationCmPerSecSq.Y);
		const bool bBraking = FVector2D::DotProduct(HorizontalAcceleration,
			FVector2D(Sample.VelocityCmPerSec.X, Sample.VelocityCmPerSec.Y)) < 0.0f;
		const float HorizontalAccelerationLimit = bBraking
			? Capability.MaxHorizontalDecelerationCmPerSecSq
			: Capability.MaxHorizontalAccelerationCmPerSecSq;
		if (HorizontalAccelerationLimit > 0.0f)
		{
			if (HorizontalAcceleration.SizeSquared() > FMath::Square(HorizontalAccelerationLimit))
			{
				return false;
			}
		}
		if (Capability.MaxVerticalAccelerationCmPerSecSq > 0.0f)
		{
			if (FMath::Abs(Sample.AccelerationCmPerSecSq.Z)
				> Capability.MaxVerticalAccelerationCmPerSecSq)
			{
				return false;
			}
		}
		return true;
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
	ReferenceAgeSeconds = FMath::Max(Config.Mpcc.MaximumReferenceAgeSeconds, 0.05f);
	YawResponseTimeSeconds = FMath::Max(Config.Mpcc.YawResponseTimeSeconds, UE_SMALL_NUMBER);
	const bool bBuilt = MpccController.SetIntent(
		InIntent, IntentId, IntentRevision, Config, State, Capability);
	Diagnostics = MpccController.GetDiagnostics();
	if (!bBuilt)
	{
		return false;
	}
	bKinematicObstructionActive = false;
	ActiveGuidanceCorridorSegmentIndex = INDEX_NONE;
	if (!bSameIntent)
	{
		PlanTimeSeconds = 0.0f;
		PlanDistanceCm = 0.0f;
		ProgressScale = 1.0f;
		PositionReferenceCm = State.PositionCm;
		VelocityReferenceCmPerSec = State.VelocityCmPerSec;
		VelocityAccelerationCmPerSecSq = FVector::ZeroVector;
		YawReferenceState.Reset();
		LastReference = {};
		LastUpdateTimeSeconds = State.TimeSeconds;
	}
	FAircraftMotionPlanSample Projection;
	if (GetPlan().Project(State.PositionCm, PlanDistanceCm, !bSameIntent, Projection))
	{
		PlanDistanceCm = Projection.DistanceCm;
		PlanTimeSeconds = GetPlan().TimeAtDistance(PlanDistanceCm);
	}
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
	YawReferenceState.Reset();
	PlanTimeSeconds = 0.0f;
	PlanDistanceCm = 0.0f;
	ProgressScale = 1.0f;
	LastUpdateTimeSeconds = 0.0;
	ResetGuidanceBlendState();
	GuidanceRebaseOffsetSeconds = 0.0;
	ActiveGuidanceCorridorSegmentIndex = INDEX_NONE;
	bKinematicObstructionActive = false;
}

void FAircraftTrajectoryRuntime::ResetGuidanceBlendState()
{
	GuidanceBlendPhase = EGuidanceBlendPhase::None;
	GuidanceBlendWeight = 0.0f;
	GuidanceRampStartSeconds = 0.0;
}

void FAircraftTrajectoryRuntime::RebaseTime(const double TimeSeconds)
{
	MpccController.RebaseTime(TimeSeconds);
	// 引导时钟重基：与 MpccController 同语义——暂停期间仿真时钟继续走（或停走，
	// 取决于上游），恢复时刻 TimeSeconds 相对引导发布时的绝对时间产生了漂移。
	// MPCC 把自身时间戳前移 PausedDuration 来补偿；NavigationGuidance 是跨线程
	// 不可变快照不能原地改——在运行时侧累计暂停时长作为偏移，与引导时间戳比较
	// 时从当前时间里扣掉（等价于把引导时间戳前移）。
	const double PausedDuration = FMath::Max(TimeSeconds - LastUpdateTimeSeconds, 0.0);
	GuidanceRebaseOffsetSeconds += PausedDuration;
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
	// 新引导到达：清掉重基偏移（新引导按当前时钟发布）。
	GuidanceRebaseOffsetSeconds = 0.0;
	bNavigationGuidanceAvailable = Guidance.IsValid() && Guidance->IsValid();
	if (!bNavigationGuidanceAvailable)
	{
		NavigationGuidance.Reset();
		ResetGuidanceBlendState();
		NavigationGuidanceStatus.State = EAircraftNavigationGuidanceState::Braking;
		NavigationGuidanceStatus.FailureReason =
			EAircraftNavigationGuidanceFailureReason::InvalidGuidance;
		return false;
	}
	NavigationGuidance = MoveTemp(Guidance);
	if (NavigationGuidance->Mode == EAircraftNavigationGuidanceMode::Brake)
	{
		// 显式刹车指令：不软化，直接全刹并复位混合状态。
		ResetGuidanceBlendState();
		NavigationGuidanceStatus.State = EAircraftNavigationGuidanceState::Braking;
		return true;
	}
	// 有效引导：混合相位的推进由 ApplyNavigationGuidance 按帧驱动。
	NavigationGuidanceStatus.State = EAircraftNavigationGuidanceState::Applied;
	return true;
}

void FAircraftTrajectoryRuntime::SetNavigationGuidanceUnavailable(const uint64 Revision)
{
	NavigationGuidance.Reset();
	NavigationGuidanceStatus = {};
	NavigationGuidanceStatus.Revision = Revision;
	bNavigationGuidanceActive = true;
	bNavigationGuidanceAvailable = false;
	ResetGuidanceBlendState();
	NavigationGuidanceStatus.State = EAircraftNavigationGuidanceState::Braking;
	NavigationGuidanceStatus.FailureReason =
		EAircraftNavigationGuidanceFailureReason::Unavailable;
}

void FAircraftTrajectoryRuntime::ClearNavigationGuidance(const uint64 Revision)
{
	NavigationGuidance.Reset();
	NavigationGuidanceStatus = {};
	NavigationGuidanceStatus.Revision = Revision;
	bNavigationGuidanceActive = false;
	bNavigationGuidanceAvailable = false;
	ResetGuidanceBlendState();
}

bool FAircraftTrajectoryRuntime::UpdateFlightController(
	const FAircraftVehicleStateSnapshot& State,
	const FAircraftDynamicCapabilitySnapshot& Capability,
	FAircraftTrajectoryReference& OutReference)
{
	TRACE_CPUPROFILER_EVENT_SCOPE(Aircraft_MPCC_Update);
	const FAircraftAutopilotDiagnostics& ControllerDiagnostics = MpccController.GetDiagnostics();
	const double EffectiveGuidanceTimeSeconds = State.TimeSeconds - GuidanceRebaseOffsetSeconds;
	const bool bSteadyGuidanceOwnsReference = GuidanceBlendPhase == EGuidanceBlendPhase::Steady
		&& bNavigationGuidanceActive && bNavigationGuidanceAvailable
		&& NavigationGuidance.IsValid()
		&& NavigationGuidance->Mode == EAircraftNavigationGuidanceMode::TimedTrajectory
		&& NavigationGuidance->SourceIntentId == ControllerDiagnostics.ActiveIntentId
		&& NavigationGuidance->SourceIntentRevision == ControllerDiagnostics.IntentRevision
		&& NavigationGuidance->IsFresh(EffectiveGuidanceTimeSeconds);
	if (bSteadyGuidanceOwnsReference)
	{
		// Steady guidance replaces the nominal MPCC output completely. Advance the
		// shared plan cursor and diagnostics without paying for a discarded solve.
		return UpdateDeterministic(State, Capability, true, true, OutReference);
	}
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
		ShapeYawReference(State, Capability, GuidanceDeltaTime, OutReference);
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
	bKinematicObstructionActive = false;
	return UpdateDeterministic(State, Capability, true, false, OutReference);
}

bool FAircraftTrajectoryRuntime::UpdateKinematicObstructed(
	const FAircraftVehicleStateSnapshot& State,
	const FAircraftDynamicCapabilitySnapshot& Capability,
	FAircraftTrajectoryReference& OutReference)
{
	TRACE_CPUPROFILER_EVENT_SCOPE(Aircraft_Kinematic_ObstructionBrake);
	if (!MpccController.RefreshPlan(State, Capability) || !GetPlan().IsValid())
	{
		Diagnostics.bPlanValid = false;
		return false;
	}
	if (!bKinematicObstructionActive)
	{
		FAircraftMotionPlanSample Projection;
		if (GetPlan().Project(State.PositionCm, PlanDistanceCm, true, Projection))
		{
			PlanDistanceCm = Projection.DistanceCm;
			PlanTimeSeconds = GetPlan().TimeAtDistance(PlanDistanceCm);
		}
		PositionReferenceCm = State.PositionCm;
		VelocityReferenceCmPerSec = State.VelocityCmPerSec;
		VelocityAccelerationCmPerSecSq = FVector::ZeroVector;
		bKinematicObstructionActive = true;
	}

	const float DeltaTime = FMath::Clamp(
		static_cast<float>(State.TimeSeconds - LastUpdateTimeSeconds), 0.0f, 0.1f);
	LastUpdateTimeSeconds = State.TimeSeconds;
	OutReference = {};
	BuildBrakingReference(State, Capability,
		EAircraftNavigationGuidanceFailureReason::KinematicObstructed,
		DeltaTime, true, OutReference);
	const FAircraftMovementIntent& Intent = GetPlan().GetIntent();
	OutReference.YawDegrees = LastReference.bValid
		? LastReference.YawDegrees
		: State.ControlRotation.Rotator().Yaw;
	OutReference.YawRateLimitDegPerSec = Intent.Limits.MaxYawRateDegPerSec;
	ShapeYawReference(State, Capability, DeltaTime, OutReference);
	FinalizeReference(State, OutReference);
	LastReference = OutReference;
	return true;
}

bool FAircraftTrajectoryRuntime::UpdateVelocity(
	const FAircraftVehicleStateSnapshot& State,
	const FAircraftDynamicCapabilitySnapshot& Capability,
	bool bUseDynamics, float DeltaTime,
	FAircraftTrajectoryReference& OutReference)
{
	const FAircraftMovementIntent& Intent = GetPlan().GetIntent();
	FVector TargetVelocity = AircraftAutopilotDynamics::ResolveVelocityWorld(
		Intent.Velocity.VelocityCmPerSec, Intent.Velocity.Frame, State.ControlRotation);
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

	const FVector PreviousVelocity = VelocityReferenceCmPerSec;
	constexpr float MaximumVelocityIntegrationStepSeconds = 1.0f / 120.0f;
	const int32 SubstepCount = FMath::Max(
		1, FMath::CeilToInt(DeltaTime / MaximumVelocityIntegrationStepSeconds));
	const float StepSeconds = DeltaTime / static_cast<float>(SubstepCount);
	for (int32 Substep = 0; Substep < SubstepCount; ++Substep)
	{
		UpdateVelocityReferenceStep(
			TargetVelocity, StepSeconds, Intent.Limits, Capability,
			VelocityReferenceCmPerSec, VelocityAccelerationCmPerSecSq);
	}
	PositionReferenceCm += 0.5f * (PreviousVelocity + VelocityReferenceCmPerSec) * DeltaTime;
	OutReference.PositionCm = PositionReferenceCm;
	OutReference.VelocityCmPerSec = VelocityReferenceCmPerSec;
	OutReference.AccelerationCmPerSecSq = VelocityAccelerationCmPerSecSq;
	OutReference.ControlAccelerationCmPerSecSq = VelocityAccelerationCmPerSecSq;
	OutReference.DynamicsFeedForwardAccelerationCmPerSecSq = bUseDynamics
		? AircraftAutopilotDynamics::ComputeDynamicsFeedForward(VelocityReferenceCmPerSec, State.BodyRotation, Capability)
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
				if (bNavigationGuidanceActive && !GetPlan().IsContinuous())
				{
					PlanDistanceCm = Projection.DistanceCm;
					PlanTimeSeconds = GetPlan().TimeAtDistance(PlanDistanceCm);
				}
				else
				{
					PlanDistanceCm = FMath::Max(PlanDistanceCm, Projection.DistanceCm);
					PlanTimeSeconds = FMath::Max(PlanTimeSeconds,
						GetPlan().TimeAtDistance(PlanDistanceCm));
				}
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
			? AircraftAutopilotDynamics::ComputeDynamicsFeedForward(Sample.VelocityCmPerSec, State.BodyRotation, Capability)
			: FVector::ZeroVector;
		OutReference.YawDegrees = Sample.YawDegrees;
		OutReference.YawRateDegPerSec = Sample.YawRateDegPerSec;
		OutReference.YawRateLimitDegPerSec = Intent.Limits.MaxYawRateDegPerSec;
		OutReference.bPositionTrackingEnabled = true;
	}
	FinalizeReference(State, OutReference);
	FAircraftMotionPlanSample DiagnosticProjection;
	if (GetPlan().Project(State.PositionCm, PlanDistanceCm, false, DiagnosticProjection))
	{
		const FVector PositionError = State.PositionCm - DiagnosticProjection.PositionCm;
		const FVector Tangent = DiagnosticProjection.VelocityCmPerSec.GetSafeNormal();
		const FVector LagError = Tangent * FVector::DotProduct(PositionError, Tangent);
		Diagnostics.ContourErrorCm = static_cast<float>((PositionError - LagError).Size());
		Diagnostics.LagErrorCm = static_cast<float>(FVector::DotProduct(PositionError, Tangent));
		Diagnostics.CorridorViolationCm = GetPlan().ComputeCorridorViolationCm(
			State.PositionCm, PlanDistanceCm);
		Diagnostics.bCorridorViolated = Diagnostics.CorridorViolationCm > 0.0f;
		Diagnostics.PathTrackingState = Diagnostics.bCorridorViolated
			? EAircraftPathTrackingState::CorridorRecovery
			: (ProgressScale < 1.0f - UE_KINDA_SMALL_NUMBER
				? EAircraftPathTrackingState::ContourLimited
				: EAircraftPathTrackingState::Nominal);
	}
	LastNominalReference = OutReference;
	ApplyNavigationGuidance(State, Capability, DeltaTime, !bUseDynamics, OutReference);
	ShapeYawReference(State, Capability, DeltaTime, OutReference);
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
	OutReference.ValidUntilSeconds = State.TimeSeconds + ReferenceAgeSeconds;
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
	const FAircraftTrajectoryReference NominalReference = InOutReference;
	// 与引导时间戳比较的"有效当前时间"：扣除暂停累计的偏移（RebaseTime 语义）。
	const double EffectiveTimeSeconds = State.TimeSeconds - GuidanceRebaseOffsetSeconds;

	const auto FailWith = [&](const EAircraftNavigationGuidanceFailureReason Reason)
	{
		ResetGuidanceBlendState();
		BuildBrakingReference(State, Capability, Reason, DeltaTime, bKinematic, InOutReference);
	};

	if (!bNavigationGuidanceAvailable)
	{
		FailWith(NavigationGuidanceStatus.FailureReason != EAircraftNavigationGuidanceFailureReason::None
			? NavigationGuidanceStatus.FailureReason
			: EAircraftNavigationGuidanceFailureReason::Unavailable);
		return;
	}
	if (NavigationGuidance->SourceIntentId != InOutReference.IntentId
		|| NavigationGuidance->SourceIntentRevision
			!= static_cast<uint64>(InOutReference.IntentRevision))
	{
		FailWith(EAircraftNavigationGuidanceFailureReason::IntentMismatch);
		return;
	}
	if (!NavigationGuidance->IsFresh(EffectiveTimeSeconds))
	{
		FailWith(EAircraftNavigationGuidanceFailureReason::Expired);
		return;
	}
	if (NavigationGuidance->Mode == EAircraftNavigationGuidanceMode::Brake)
	{
		// 显式刹车指令：主动指令不软化，直接全刹。
		ResetGuidanceBlendState();
		BuildBrakingReference(State, Capability,
			EAircraftNavigationGuidanceFailureReason::None,
			DeltaTime, bKinematic, InOutReference);
		return;
	}

	FAircraftNavigationGuidanceSample Sample;
	if (!NavigationGuidance->Evaluate(EffectiveTimeSeconds, Sample))
	{
		FailWith(EAircraftNavigationGuidanceFailureReason::InvalidGuidance);
		return;
	}
	if (!IsGuidanceSampleWithinCapability(Sample, Capability))
	{
		FailWith(EAircraftNavigationGuidanceFailureReason::InvalidGuidance);
		return;
	}

	// ── 混合应用 ──────────────────────────────────────────────
	if (GuidanceBlendPhase == EGuidanceBlendPhase::None)
	{
		// 首次应用：渐入基准取引导发布时刻——引导从发布到被本帧
		// 消费的时间差已在窗口内计入，消费延迟不会重置渐变进度。
		GuidanceBlendPhase = EGuidanceBlendPhase::RampingIn;
		GuidanceBlendWeight = 0.0f;
		GuidanceRampStartSeconds = NavigationGuidance->GeneratedAtSeconds;
	}
	if (GuidanceBlendPhase == EGuidanceBlendPhase::RampingIn)
	{
		GuidanceBlendWeight = FMath::Min(
			static_cast<float>((EffectiveTimeSeconds - GuidanceRampStartSeconds)
				/ GuidanceBlendRampSeconds),
			1.0f);
		if (GuidanceBlendWeight >= 1.0f)
		{
			GuidanceBlendPhase = EGuidanceBlendPhase::Steady;
		}
	}

	const float Weight = FMath::Clamp(GuidanceBlendWeight, 0.0f, 1.0f);
	if (Weight >= 1.0f)
	{
		// Steady：等同原覆写语义。
		InOutReference.PositionCm = Sample.PositionCm;
		InOutReference.VelocityCmPerSec = Sample.VelocityCmPerSec;
		InOutReference.AccelerationCmPerSecSq = Sample.AccelerationCmPerSecSq;
		InOutReference.ControlAccelerationCmPerSecSq = Sample.AccelerationCmPerSecSq;
		InOutReference.bPositionTrackingEnabled = true;
	}
	else
	{
		// RampingIn：nominal（进入本函数时的参考）与引导样本交叉渐变。
		InOutReference.PositionCm = FMath::Lerp(
			InOutReference.PositionCm, Sample.PositionCm, Weight);
		InOutReference.VelocityCmPerSec = FMath::Lerp(
			InOutReference.VelocityCmPerSec, Sample.VelocityCmPerSec, Weight);
		InOutReference.AccelerationCmPerSecSq = FMath::Lerp(
			InOutReference.AccelerationCmPerSecSq, Sample.AccelerationCmPerSecSq, Weight);
		InOutReference.ControlAccelerationCmPerSecSq = InOutReference.AccelerationCmPerSecSq;
		// 前半程让 nominal 位置环主导，避免混合位置参考在两个目标间拉扯。
		InOutReference.bPositionTrackingEnabled = Weight > 0.5f
			|| InOutReference.bPositionTrackingEnabled;
	}

	if (GetPlan().HasCorridor())
	{
		constexpr float CorridorToleranceCm = 0.1f;
		constexpr float MaximumValidationHorizonSeconds = 0.25f;
		const float ValidationHorizonSeconds = FMath::Clamp(
			static_cast<float>(NavigationGuidance->ValidUntilSeconds - EffectiveTimeSeconds),
			FMath::Max(DeltaTime, UE_SMALL_NUMBER), MaximumValidationHorizonSeconds);
		const auto ComputeReferenceViolation = [&](
			const FAircraftTrajectoryReference& Reference, int32& InOutActiveSegment)
		{
			const auto PredictPosition = [&](const float TimeSeconds)
			{
				return Reference.PositionCm
					+ Reference.VelocityCmPerSec * TimeSeconds
					+ 0.5f * Reference.AccelerationCmPerSecSq
						* FMath::Square(TimeSeconds);
			};
			float MaximumViolationCm = 0.0f;
			const auto ValidateLine = [&](const FVector& StartCm, const FVector& EndCm,
				const float ChordDeviationCm)
			{
				TArray<int32> CandidateIndices;
				if (!GetPlan().BuildContinuousCorridorCandidates(StartCm, EndCm,
					InOutActiveSegment, CandidateIndices)
					|| !GetPlan().IsCorridorLineContinuouslyCovered(StartCm, EndCm,
						CandidateIndices, ChordDeviationCm))
				{
					return false;
				}
				MaximumViolationCm = FMath::Max(MaximumViolationCm,
					FMath::Max(GetPlan().ComputeCorridorUnionViolationCm(StartCm, CandidateIndices),
						GetPlan().ComputeCorridorUnionViolationCm(EndCm, CandidateIndices)));
				return true;
			};
			if (!ValidateLine(State.PositionCm, PredictPosition(0.0f), 0.0f))
			{
				return TNumericLimits<float>::Max();
			}
			struct FValidationInterval
			{
				float StartSeconds;
				float EndSeconds;
				int32 Depth;
			};
			TArray<FValidationInterval, TInlineAllocator<16>> Pending;
			Pending.Add({ 0.0f, ValidationHorizonSeconds, 0 });
			while (!Pending.IsEmpty())
			{
				const FValidationInterval Interval = Pending.Pop(EAllowShrinking::No);
				const float DurationSeconds = Interval.EndSeconds - Interval.StartSeconds;
				const float ChordDeviationCm = static_cast<float>(
					Reference.AccelerationCmPerSecSq.Size())
					* FMath::Square(DurationSeconds) * 0.125f;
				if ((DurationSeconds > 0.1f || ChordDeviationCm > 0.05f)
					&& Interval.Depth < 12)
				{
					const float MiddleSeconds = 0.5f
						* (Interval.StartSeconds + Interval.EndSeconds);
					Pending.Add({ MiddleSeconds, Interval.EndSeconds, Interval.Depth + 1 });
					Pending.Add({ Interval.StartSeconds, MiddleSeconds, Interval.Depth + 1 });
					continue;
				}
				if ((DurationSeconds > 0.1f || ChordDeviationCm > 0.05f)
					|| !ValidateLine(PredictPosition(Interval.StartSeconds),
						PredictPosition(Interval.EndSeconds), ChordDeviationCm))
				{
					return TNumericLimits<float>::Max();
				}
			}
			return MaximumViolationCm;
		};

		int32 NominalActiveSegment = ActiveGuidanceCorridorSegmentIndex;
		const float NominalViolationCm = ComputeReferenceViolation(
			NominalReference, NominalActiveSegment);
		if (NominalViolationCm > CorridorToleranceCm)
		{
			BuildBrakingReference(State, Capability,
				EAircraftNavigationGuidanceFailureReason::InvalidGuidance,
				DeltaTime, bKinematic, InOutReference);
			return;
		}

		const FAircraftTrajectoryReference GuidedReference = InOutReference;
		int32 GuidedActiveSegment = ActiveGuidanceCorridorSegmentIndex;
		const float UnconstrainedViolationCm = ComputeReferenceViolation(
			GuidedReference, GuidedActiveSegment);
		if (UnconstrainedViolationCm > CorridorToleranceCm)
		{
			float SafeAlpha = 0.0f;
			float UnsafeAlpha = 1.0f;
			for (int32 Iteration = 0; Iteration < 10; ++Iteration)
			{
				const float CandidateAlpha = 0.5f * (SafeAlpha + UnsafeAlpha);
				FAircraftTrajectoryReference CandidateReference = NominalReference;
				CandidateReference.PositionCm = FMath::Lerp(NominalReference.PositionCm,
					GuidedReference.PositionCm, CandidateAlpha);
				CandidateReference.VelocityCmPerSec = FMath::Lerp(
					NominalReference.VelocityCmPerSec,
					GuidedReference.VelocityCmPerSec, CandidateAlpha);
				CandidateReference.AccelerationCmPerSecSq = FMath::Lerp(
					NominalReference.AccelerationCmPerSecSq,
					GuidedReference.AccelerationCmPerSecSq, CandidateAlpha);
				int32 CandidateActiveSegment = ActiveGuidanceCorridorSegmentIndex;
				const float CandidateViolationCm = ComputeReferenceViolation(
					CandidateReference, CandidateActiveSegment);
				if (CandidateViolationCm <= CorridorToleranceCm)
				{
					SafeAlpha = CandidateAlpha;
				}
				else
				{
					UnsafeAlpha = CandidateAlpha;
				}
			}
			InOutReference.PositionCm = FMath::Lerp(
				NominalReference.PositionCm, GuidedReference.PositionCm, SafeAlpha);
			InOutReference.VelocityCmPerSec = FMath::Lerp(
				NominalReference.VelocityCmPerSec, GuidedReference.VelocityCmPerSec, SafeAlpha);
			InOutReference.AccelerationCmPerSecSq = FMath::Lerp(
				NominalReference.AccelerationCmPerSecSq,
				GuidedReference.AccelerationCmPerSecSq, SafeAlpha);
			InOutReference.ControlAccelerationCmPerSecSq = FMath::Lerp(
				NominalReference.ControlAccelerationCmPerSecSq,
				GuidedReference.ControlAccelerationCmPerSecSq, SafeAlpha);
			InOutReference.bPositionTrackingEnabled = SafeAlpha > 0.5f
				? GuidedReference.bPositionTrackingEnabled
				: NominalReference.bPositionTrackingEnabled;
			Diagnostics.PathTrackingState = EAircraftPathTrackingState::CorridorConstrained;
		}
		Diagnostics.CorridorViolationCm = ComputeReferenceViolation(
			InOutReference, ActiveGuidanceCorridorSegmentIndex);
		Diagnostics.bCorridorViolated = Diagnostics.CorridorViolationCm > CorridorToleranceCm;
	}
	// 前馈按混合后的实际速度计算（不是样本速度）。
	InOutReference.DynamicsFeedForwardAccelerationCmPerSecSq =
		AircraftAutopilotDynamics::ComputeDynamicsFeedForward(
			InOutReference.VelocityCmPerSec, State.BodyRotation, Capability);

	// ── 引导激活期的航向重解析 ─────────────────────────────────
	// nominal 的 yaw 采样自计划进度，而避障期间轮廓误差会把 ProgressScale 压到趋 0
	// （计划时间冻结、YawRate 同步缩放）——飞机实际在绕行，参考航向却停在旧值，
	// heading 与实际运动脱钩。此处按混合后的实际速度/位置 + 意图 heading 目标
	// 实时重解析，恢复 heading 权威与实际运动的一致性。
	InOutReference.YawDegrees = FAircraftMotionPlan::ResolveYaw(
		GetPlan().GetIntent().Heading,
		InOutReference.PositionCm, InOutReference.VelocityCmPerSec,
		InOutReference.YawDegrees);
	// 最终偏航角、速率和加速度由 ShapeYawReference 在所有 Guidance 合成后
	// 一次性生成，禁止在覆盖层重新制造不一致的角度/速率组合。
	InOutReference.YawRateDegPerSec = 0.0f;
	InOutReference.YawAccelerationDegPerSecSq = 0.0f;

	InOutReference.ValidUntilSeconds = FMath::Min(
		InOutReference.ValidUntilSeconds, NavigationGuidance->ValidUntilSeconds);
	NavigationGuidanceStatus.State = EAircraftNavigationGuidanceState::Applied;
	NavigationGuidanceStatus.FailureReason = EAircraftNavigationGuidanceFailureReason::None;
}

void FAircraftTrajectoryRuntime::ShapeYawReference(
	const FAircraftVehicleStateSnapshot& State,
	const FAircraftDynamicCapabilitySnapshot& Capability,
	const float DeltaTime,
	FAircraftTrajectoryReference& InOutReference)
{
	const FAircraftMovementIntent& Intent = GetPlan().GetIntent();
	const float MeasuredYawDegrees = State.ControlRotation.Rotator().Yaw;
	const FVector AngularVelocityWorldRadPerSec = State.BodyRotation.RotateVector(
		State.AngularVelocityBodyRadPerSec);
	const FVector ForwardAxisBody = Capability.AircraftToBodyRotation.RotateVector(
		FVector::ForwardVector);
	const FVector RightAxisBody = Capability.AircraftToBodyRotation.RotateVector(
		FVector::RightVector);
	const float MeasuredYawRateDegPerSec =
		AircraftAttitudeReference::GetPlanarHeadingRateDegreesPerSecond(
			State.BodyRotation, AngularVelocityWorldRadPerSec,
			ForwardAxisBody, RightAxisBody);
	InOutReference.YawRateLimitDegPerSec = Intent.Limits.MaxYawRateDegPerSec;

	if (DeltaTime <= UE_SMALL_NUMBER)
	{
		InOutReference.YawDegrees = FRotator::NormalizeAxis(MeasuredYawDegrees);
		InOutReference.YawRateDegPerSec = FMath::Clamp(MeasuredYawRateDegPerSec,
			-Intent.Limits.MaxYawRateDegPerSec, Intent.Limits.MaxYawRateDegPerSec);
		InOutReference.YawAccelerationDegPerSecSq = 0.0f;
		return;
	}

	FAircraftYawReferenceLimits Limits;
	Limits.MaxRateDegPerSec = Intent.Limits.MaxYawRateDegPerSec;
	Limits.MaxAccelerationDegPerSecSq = Intent.Limits.MaxYawAccelerationDegPerSecSq;
	Limits.MaxJerkDegPerSecCubed = Intent.Limits.MaxYawJerkDegPerSecCubed;
	Limits.ResponseTimeSeconds = YawResponseTimeSeconds;
	const bool bUpdated = Intent.Heading.Mode == EAircraftHeadingMode::YawRate
		? FAircraftYawReferenceDynamics::UpdateRateCommand(
			Intent.Heading.YawRateDegPerSec,
			MeasuredYawDegrees, MeasuredYawRateDegPerSec,
			DeltaTime, Limits, YawReferenceState)
		: FAircraftYawReferenceDynamics::UpdateAngleCommand(
			InOutReference.YawDegrees,
			MeasuredYawDegrees, MeasuredYawRateDegPerSec,
			DeltaTime, Limits, YawReferenceState);
	if (!bUpdated)
	{
		YawReferenceState.Reset();
		InOutReference.YawDegrees = FRotator::NormalizeAxis(MeasuredYawDegrees);
		InOutReference.YawRateDegPerSec = 0.0f;
		InOutReference.YawAccelerationDegPerSecSq = 0.0f;
		return;
	}
	InOutReference.YawDegrees = YawReferenceState.YawDegrees;
	InOutReference.YawRateDegPerSec = YawReferenceState.RateDegPerSec;
	InOutReference.YawAccelerationDegPerSecSq =
		YawReferenceState.AccelerationDegPerSecSq;
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
	InOutReference.ValidUntilSeconds = State.TimeSeconds + ReferenceAgeSeconds;
	NavigationGuidanceStatus.State = EAircraftNavigationGuidanceState::Braking;
	NavigationGuidanceStatus.FailureReason = FailureReason;
}

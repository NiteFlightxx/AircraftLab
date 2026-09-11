#include "AircraftAutopilot/AircraftTrajectoryRuntime.h"

#include "ProfilingDebugging/CpuProfilerTrace.h"

namespace
{
	// 引导覆盖层混合参数：激活渐入时长与退场保持窗口（秒）。
	// 与 GuidanceValiditySeconds(0.25) 同量级——保持窗口覆盖一个引导发布周期，
	// 让 10Hz 发布者的一次瞬态丢失不触发全刹。
	constexpr float GuidanceBlendRampSeconds = 0.2f;
	constexpr float GuidanceHoldSeconds = 0.25f;

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
	ResetGuidanceBlendState();
	GuidanceRebaseOffsetSeconds = 0.0;
}

void FAircraftTrajectoryRuntime::ResetGuidanceBlendState()
{
	GuidanceBlendPhase = EGuidanceBlendPhase::None;
	GuidanceBlendWeight = 0.0f;
	LastAppliedGuidanceVelocityCmPerSec = FVector::ZeroVector;
	LastAppliedGuidanceAccelerationCmPerSecSq = FVector::ZeroVector;
	GuidanceHoldUntilSeconds = 0.0;
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
		EnterGuidanceHoldingOrBrake(
			EAircraftNavigationGuidanceFailureReason::InvalidGuidance);
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
	// 有效引导：混合相位的推进由 ApplyNavigationGuidance 按帧驱动
	// （None/Holding → RampingIn，基准 = 引导发布时刻；RampingIn/Steady 跨发布保持）。
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
	EnterGuidanceHoldingOrBrake(
		EAircraftNavigationGuidanceFailureReason::Unavailable);
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

void FAircraftTrajectoryRuntime::EnterGuidanceHoldingOrBrake(
	const EAircraftNavigationGuidanceFailureReason FailureReason)
{
	NavigationGuidanceStatus.FailureReason = FailureReason;
	if (GuidanceBlendWeight > 0.0f
		&& (GuidanceBlendPhase == EGuidanceBlendPhase::Steady
			|| GuidanceBlendPhase == EGuidanceBlendPhase::RampingIn))
	{
		// 曾有效混合过：进入 Holding 退场保持，而非立即全刹。
		// 窗口截止时刻在 BuildHoldingReference 首帧锚定（此处拿不到仿真时间）。
		GuidanceBlendPhase = EGuidanceBlendPhase::Holding;
		GuidanceHoldUntilSeconds = -1.0;
		NavigationGuidanceStatus.State = EAircraftNavigationGuidanceState::Braking;
	}
	else
	{
		// 从未混合成功：保持原有直接刹车语义。
		ResetGuidanceBlendState();
		NavigationGuidanceStatus.State = EAircraftNavigationGuidanceState::Braking;
	}
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
	return UpdateDeterministic(State, Capability, true, false, OutReference);
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
	// 与引导时间戳比较的"有效当前时间"：扣除暂停累计的偏移（RebaseTime 语义）。
	const double EffectiveTimeSeconds = State.TimeSeconds - GuidanceRebaseOffsetSeconds;

	const auto FailWith = [&](const EAircraftNavigationGuidanceFailureReason Reason)
	{
		if (GuidanceBlendPhase == EGuidanceBlendPhase::Holding)
		{
			// 已在退场保持中：继续衰减（窗口截止在 BuildHoldingReference 首帧锚定）。
			BuildHoldingReference(State, Capability, Reason, DeltaTime, bKinematic, InOutReference);
		}
		else if (GuidanceBlendWeight > 0.0f)
		{
			// 曾有效混合过：进入 Holding 软退场而非立即全刹。
			GuidanceBlendPhase = EGuidanceBlendPhase::Holding;
			GuidanceHoldUntilSeconds = -1.0; // BuildHoldingReference 首帧锚定
			BuildHoldingReference(State, Capability, Reason, DeltaTime, bKinematic, InOutReference);
		}
		else
		{
			BuildBrakingReference(State, Capability, Reason, DeltaTime, bKinematic, InOutReference);
		}
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
		NavigationGuidanceStatus.State = EAircraftNavigationGuidanceState::Inactive;
		NavigationGuidanceStatus.FailureReason =
			EAircraftNavigationGuidanceFailureReason::IntentMismatch;
		// 意图切换：混合状态无意义，复位（nominal 已是新意图的，直接跟随）。
		ResetGuidanceBlendState();
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
	if (GuidanceBlendPhase == EGuidanceBlendPhase::None
		|| GuidanceBlendPhase == EGuidanceBlendPhase::Holding)
	{
		// 首次应用或 Holding 恢复：渐入基准取引导发布时刻——引导从发布到被本帧
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
	// 记录最后应用的引导速度/加速度，供退场保持。
	LastAppliedGuidanceVelocityCmPerSec = Sample.VelocityCmPerSec;
	LastAppliedGuidanceAccelerationCmPerSecSq = Sample.AccelerationCmPerSecSq;
	// 前馈按混合后的实际速度计算（不是样本速度）。
	InOutReference.DynamicsFeedForwardAccelerationCmPerSecSq =
		ComputeDynamicsFeedForward(InOutReference.VelocityCmPerSec, State.BodyRotation, Capability);

	// ── 引导激活期的航向重解析 ─────────────────────────────────
	// nominal 的 yaw 采样自计划进度，而避障期间轮廓误差会把 ProgressScale 压到趋 0
	// （计划时间冻结、YawRate 同步缩放）——飞机实际在绕行，参考航向却停在旧值，
	// heading 与实际运动脱钩。此处按混合后的实际速度/位置 + 意图 heading 目标
	// 实时重解析，恢复 heading 权威与实际运动的一致性。
	InOutReference.YawDegrees = FAircraftMotionPlan::ResolveYaw(
		GetPlan().GetIntent().Heading,
		InOutReference.PositionCm, InOutReference.VelocityCmPerSec,
		InOutReference.YawDegrees);
	// 航向角变了，速率前馈按新旧角差重估（保持与 ApplyYawConstraints 的
	// 梯形语义一致：角差/时间，限幅到意图限速内）。
	const float ResolvedYawDelta = FMath::FindDeltaAngleDegrees(
		State.ControlRotation.Rotator().Yaw, InOutReference.YawDegrees);
	if (InOutReference.YawRateLimitDegPerSec > UE_SMALL_NUMBER)
	{
		InOutReference.YawRateDegPerSec = FMath::Clamp(
			ResolvedYawDelta / FMath::Max(DeltaTime, UE_SMALL_NUMBER),
			-InOutReference.YawRateLimitDegPerSec,
			InOutReference.YawRateLimitDegPerSec);
	}
	else
	{
		InOutReference.YawRateDegPerSec = 0.0f;
	}
	InOutReference.YawAccelerationDegPerSecSq = 0.0f;

	InOutReference.ValidUntilSeconds = FMath::Min(
		InOutReference.ValidUntilSeconds, NavigationGuidance->ValidUntilSeconds);
	NavigationGuidanceStatus.State = EAircraftNavigationGuidanceState::Applied;
	NavigationGuidanceStatus.FailureReason = EAircraftNavigationGuidanceFailureReason::None;
}

void FAircraftTrajectoryRuntime::BuildHoldingReference(
	const FAircraftVehicleStateSnapshot& State,
	const FAircraftDynamicCapabilitySnapshot& Capability,
	const EAircraftNavigationGuidanceFailureReason FailureReason,
	const float DeltaTime, const bool bKinematic,
	FAircraftTrajectoryReference& InOutReference)
{
	// Holding：保持最后已应用的引导速度并衰减——失败路径的中间档，
	// 避免瞬态失败（卡帧/单帧坏样本）立即触发全力刹车。
	const FVector HoldVelocity = LastAppliedGuidanceVelocityCmPerSec;
	const double EffectiveTimeSeconds = State.TimeSeconds - GuidanceRebaseOffsetSeconds;

	// 首帧锚定保持窗口（进入 Holding 时刻拿不到仿真时间，用 -1 哨兵延迟锚定）。
	if (GuidanceHoldUntilSeconds < 0.0)
	{
		GuidanceHoldUntilSeconds = EffectiveTimeSeconds + GuidanceHoldSeconds;
	}

	if (EffectiveTimeSeconds >= GuidanceHoldUntilSeconds)
	{
		// 保持窗口耗尽：收敛到常规全刹。
		GuidanceBlendPhase = EGuidanceBlendPhase::None;
		GuidanceBlendWeight = 0.0f;
		BuildBrakingReference(State, Capability, FailureReason, DeltaTime, bKinematic, InOutReference);
		return;
	}

	// 窗口内：按能力做温和减速（一半减速度），保持位置追踪连续。
	const FVector2D HorizontalVelocity(HoldVelocity.X, HoldVelocity.Y);
	const float HorizontalDeceleration = FMath::Max(
		Capability.MaxHorizontalDecelerationCmPerSecSq * 0.5f, 1.0f);
	const FVector2D HorizontalAcceleration = HorizontalVelocity.IsNearlyZero()
		? FVector2D::ZeroVector
		: -HorizontalVelocity.GetSafeNormal() * HorizontalDeceleration;
	const float VerticalAcceleration = FMath::Max(
		Capability.MaxVerticalAccelerationCmPerSecSq * 0.5f, 1.0f);
	const float VerticalBrakingAcceleration = FMath::IsNearlyZero(HoldVelocity.Z)
		? 0.0f
		: -FMath::Sign(HoldVelocity.Z) * VerticalAcceleration;
	const FVector BrakingAcceleration(
		HorizontalAcceleration.X, HorizontalAcceleration.Y, VerticalBrakingAcceleration);

	const float StepSeconds = FMath::Clamp(DeltaTime, 0.0f, 0.1f);
	FVector NextVelocity = HoldVelocity + BrakingAcceleration * StepSeconds;
	for (int32 Axis = 0; Axis < 3; ++Axis)
	{
		if (HoldVelocity[Axis] * NextVelocity[Axis] <= 0.0f)
		{
			NextVelocity[Axis] = 0.0f;
		}
	}
	InOutReference.PositionCm = State.PositionCm
		+ 0.5f * (HoldVelocity + NextVelocity) * StepSeconds;
	InOutReference.VelocityCmPerSec = NextVelocity;
	InOutReference.AccelerationCmPerSecSq = BrakingAcceleration;
	InOutReference.ControlAccelerationCmPerSecSq = BrakingAcceleration;
	InOutReference.DynamicsFeedForwardAccelerationCmPerSecSq = FVector::ZeroVector;
	InOutReference.bPositionTrackingEnabled = true;
	InOutReference.GeneratedAtSeconds = State.TimeSeconds;
	InOutReference.ValidUntilSeconds = State.TimeSeconds + 0.15;
	NavigationGuidanceStatus.State = EAircraftNavigationGuidanceState::Braking;
	NavigationGuidanceStatus.FailureReason = FailureReason;
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

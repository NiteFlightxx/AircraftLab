#include "AircraftAutopilot/AutopilotComponent.h"

#include "AircraftAutopilot/AircraftMotionPlan.h"
#include "AircraftDiagnostics/AircraftDebug.h"
#include "AircraftDiagnostics/AircraftDebugSettings.h"
#include "GameFramework/Actor.h"

#include UE_INLINE_GENERATED_CPP_BY_NAME(AutopilotComponent)

UAutopilotComponent::UAutopilotComponent()
{
	PrimaryComponentTick.bCanEverTick = true;
	PrimaryComponentTick.TickGroup = TG_PrePhysics;
}

void UAutopilotComponent::BeginPlay()
{
	Super::BeginPlay();
	ResolveFlightController();
	if (IAircraftFlightControllerInterface* Controller = GetFlightController())
	{
		Controller->SetAircraftMovementIntentProvider(this);
	}
}

void UAutopilotComponent::EndPlay(const EEndPlayReason::Type EndPlayReason)
{
	ReleaseFlightControl();
	if (IAircraftFlightControllerInterface* Controller = GetFlightController())
	{
		Controller->SetAircraftMovementIntentProvider(nullptr);
	}
	FlightControllerComponent = nullptr;
	Super::EndPlay(EndPlayReason);
}

IAircraftFlightControllerInterface* UAutopilotComponent::GetFlightController() const
{
	return Cast<IAircraftFlightControllerInterface>(FlightControllerComponent);
}

void UAutopilotComponent::ResolveFlightController()
{
	if (GetFlightController() || !GetOwner())
	{
		return;
	}
	TInlineComponentArray<UActorComponent*> Components(GetOwner());
	for (UActorComponent* Component : Components)
	{
		if (Component && Component->Implements<UAircraftFlightControllerInterface>())
		{
			FlightControllerComponent = Component;
			GetFlightController()->SetAircraftMovementIntentProvider(this);
			return;
		}
	}
}

bool UAutopilotComponent::AcquireFlightControl()
{
	if (bControlClaimed)
	{
		return GetFlightController() != nullptr;
	}
	ResolveFlightController();
	if (IAircraftFlightControllerInterface* Controller = GetFlightController())
	{
		FlightModeBeforeActivation = Controller->ActivateAircraftAutopilotControl();
		bControlClaimed = true;
		return true;
	}
	return false;
}

void UAutopilotComponent::ReleaseFlightControl()
{
	if (!bControlClaimed)
	{
		return;
	}
	if (IAircraftFlightControllerInterface* Controller = GetFlightController())
	{
		Controller->DeactivateAircraftAutopilotControl(FlightModeBeforeActivation);
	}
	bControlClaimed = false;
}

FAircraftMovementIntent UAutopilotComponent::BuildIntent(
	EAircraftMovementIntentType Type,
	const FAircraftMovementIntentSettings& Settings,
	const FAircraftCompletionPolicy* Completion)
{
	FAircraftMovementIntent Intent;
	Intent.Type = Type;
	Intent.Limits = Settings.Limits;
	Intent.bHasRequestedMotionLimits = Settings.bOverrideMotionLimits;
	Intent.Heading = Settings.Heading;
	Intent.TimeoutSeconds = Settings.TimeoutSeconds;
	if (Completion)
	{
		Intent.Completion = *Completion;
	}
	return Intent;
}

FAircraftMovementIntentHandle UAutopilotComponent::SubmitHoldIntent(
	const FAircraftHoldIntent& Hold,
	const FAircraftMovementIntentSettings& Settings,
	const FAircraftCompletionPolicy& Completion)
{
	FAircraftMovementIntent Intent = BuildIntent(
		EAircraftMovementIntentType::Hold, Settings, &Completion);
	Intent.Hold = Hold;
	return SubmitIntent(Intent);
}

bool UAutopilotComponent::UpdateHoldIntent(
	FAircraftMovementIntentHandle Handle,
	const FAircraftHoldIntent& Hold,
	const FAircraftMovementIntentSettings& Settings,
	const FAircraftCompletionPolicy& Completion)
{
	FAircraftMovementIntent Intent = BuildIntent(
		EAircraftMovementIntentType::Hold, Settings, &Completion);
	Intent.Hold = Hold;
	return UpdateIntent(Handle, Intent);
}

FAircraftMovementIntentHandle UAutopilotComponent::SubmitVelocityIntent(
	const FAircraftVelocityIntent& Velocity,
	const FAircraftMovementIntentSettings& Settings)
{
	FAircraftMovementIntent Intent = BuildIntent(
		EAircraftMovementIntentType::Velocity, Settings);
	Intent.Velocity = Velocity;
	return SubmitIntent(Intent);
}

bool UAutopilotComponent::UpdateVelocityIntent(
	FAircraftMovementIntentHandle Handle,
	const FAircraftVelocityIntent& Velocity,
	const FAircraftMovementIntentSettings& Settings)
{
	FAircraftMovementIntent Intent = BuildIntent(
		EAircraftMovementIntentType::Velocity, Settings);
	Intent.Velocity = Velocity;
	return UpdateIntent(Handle, Intent);
}

FAircraftMovementIntentHandle UAutopilotComponent::SubmitRouteIntent(
	const FAircraftRouteIntent& Route,
	const FAircraftMovementIntentSettings& Settings,
	const FAircraftCompletionPolicy& Completion)
{
	FAircraftMovementIntent Intent = BuildIntent(
		EAircraftMovementIntentType::Route, Settings, &Completion);
	Intent.Route = Route;
	return SubmitIntent(Intent);
}

bool UAutopilotComponent::UpdateRouteIntent(
	FAircraftMovementIntentHandle Handle,
	const FAircraftRouteIntent& Route,
	const FAircraftMovementIntentSettings& Settings,
	const FAircraftCompletionPolicy& Completion)
{
	FAircraftMovementIntent Intent = BuildIntent(
		EAircraftMovementIntentType::Route, Settings, &Completion);
	Intent.Route = Route;
	return UpdateIntent(Handle, Intent);
}

FAircraftSafeCorridorBuildResult UAutopilotComponent::BuildSafeCorridorFromPathPoints(
	const TArray<FVector>& PathPointsCm,
	const FAircraftSafeCorridorBuildSettings& Settings,
	FAircraftRouteIntent& OutRoute)
{
	ResolveFlightController();
	FAircraftAutopilotRuntimeConfig RuntimeConfig;
	IAircraftFlightControllerInterface* const Controller = GetFlightController();
	if (!Controller || !Controller->GetAircraftAutopilotRuntimeConfig(RuntimeConfig))
	{
		OutRoute = {};
		FAircraftSafeCorridorBuildResult Result;
		Result.Status = EAircraftSafeCorridorBuildStatus::RuntimeConfigUnavailable;
		return Result;
	}
	return FAircraftSafeCorridorBuilder::BuildOpenPolyline(
		PathPointsCm, Settings, RuntimeConfig.Path, OutRoute);
}

FAircraftMovementIntentHandle UAutopilotComponent::SubmitOrbitIntent(
	const FAircraftOrbitIntent& Orbit,
	const FAircraftMovementIntentSettings& Settings)
{
	FAircraftMovementIntent Intent = BuildIntent(
		EAircraftMovementIntentType::Orbit, Settings);
	Intent.Orbit = Orbit;
	return SubmitIntent(Intent);
}

bool UAutopilotComponent::UpdateOrbitIntent(
	FAircraftMovementIntentHandle Handle,
	const FAircraftOrbitIntent& Orbit,
	const FAircraftMovementIntentSettings& Settings)
{
	FAircraftMovementIntent Intent = BuildIntent(
		EAircraftMovementIntentType::Orbit, Settings);
	Intent.Orbit = Orbit;
	return UpdateIntent(Handle, Intent);
}

FAircraftMovementIntentHandle UAutopilotComponent::SubmitTimedTrajectoryIntent(
	const FAircraftTimedTrajectoryIntent& TimedTrajectory,
	const FAircraftMovementIntentSettings& Settings,
	const FAircraftCompletionPolicy& Completion)
{
	FAircraftMovementIntent Intent = BuildIntent(
		EAircraftMovementIntentType::TimedTrajectory, Settings, &Completion);
	Intent.TimedTrajectory = TimedTrajectory;
	return SubmitIntent(Intent);
}

bool UAutopilotComponent::UpdateTimedTrajectoryIntent(
	FAircraftMovementIntentHandle Handle,
	const FAircraftTimedTrajectoryIntent& TimedTrajectory,
	const FAircraftMovementIntentSettings& Settings,
	const FAircraftCompletionPolicy& Completion)
{
	FAircraftMovementIntent Intent = BuildIntent(
		EAircraftMovementIntentType::TimedTrajectory, Settings, &Completion);
	Intent.TimedTrajectory = TimedTrajectory;
	return UpdateIntent(Handle, Intent);
}

FAircraftMovementIntentHandle UAutopilotComponent::SubmitIntent(
	const FAircraftMovementIntent& Intent)
{
	if (!bActive || !Intent.IsValid())
	{
		CurrentResult = {};
		CurrentResult.Status = EAircraftMovementIntentStatus::Failed;
		CurrentResult.FailureReason = bActive
			? EAircraftMovementFailureReason::InvalidIntent
			: EAircraftMovementFailureReason::AutopilotInactive;
		OnMovementIntentChanged.Broadcast(CurrentResult);
		return {};
	}
	if (ActiveHandle.IsValid())
	{
		Finish(EAircraftMovementIntentStatus::Interrupted,
			EAircraftMovementFailureReason::Replaced);
	}
	ClearAutomaticContinuation();
	AcquireFlightControl();
	SourceIntent = Intent;
	ResolvedIntent = Intent;
	ActiveHandle.Id = NextIntentId++;
	++IntentRevision;
	ElapsedSeconds = 0.0f;
	DiagnosticLogAccumulatorSeconds = 0.0f;
	StableTimeSeconds = 0.0f;
	InitialDistanceToTargetCm = -1.0f;
	CurrentResult = {};
	CurrentResult.Handle = ActiveHandle;
	CurrentResult.Status = EAircraftMovementIntentStatus::Accepted;
	ResolveActorTargets();
	OnMovementIntentChanged.Broadcast(CurrentResult);
	return ActiveHandle;
}

bool UAutopilotComponent::UpdateIntent(
	FAircraftMovementIntentHandle Handle, const FAircraftMovementIntent& Intent)
{
	if (!bActive || Handle != ActiveHandle || !Intent.IsValid())
	{
		return false;
	}
	SourceIntent = Intent;
	ResolvedIntent = Intent;
	++IntentRevision;
	StableTimeSeconds = 0.0f;
	InitialDistanceToTargetCm = -1.0f;
	CurrentResult.Status = EAircraftMovementIntentStatus::Accepted;
	ResolveActorTargets();
	OnMovementIntentChanged.Broadcast(CurrentResult);
	return true;
}

bool UAutopilotComponent::CancelMovementIntent(FAircraftMovementIntentHandle Handle)
{
	if (Handle == ActiveHandle)
	{
		Finish(EAircraftMovementIntentStatus::Cancelled,
			EAircraftMovementFailureReason::CancelledByCaller);
		return true;
	}
	if (Handle == AutomaticContinuationHandle)
	{
		ClearAutomaticContinuation();
		++IntentRevision;
		return true;
	}
	return false;
}

void UAutopilotComponent::SetAutopilotActive(bool bInActive)
{
	if (bActive == bInActive)
	{
		if (bActive)
		{
			AcquireFlightControl();
		}
		return;
	}
	bActive = bInActive;
	if (bActive)
	{
		AcquireFlightControl();
	}
	else
	{
		if (ActiveHandle.IsValid())
		{
			Finish(EAircraftMovementIntentStatus::Failed,
				EAircraftMovementFailureReason::AutopilotInactive);
		}
		ClearAutomaticContinuation();
		++IntentRevision;
		ReleaseFlightControl();
	}
}

void UAutopilotComponent::ResolveActorTargets()
{
	const FAircraftMovementIntent Previous = ResolvedIntent;
	ResolvedIntent = SourceIntent;
	if (IsValid(SourceIntent.Hold.TargetActor))
	{
		ResolvedIntent.Hold.PositionCm = SourceIntent.Hold.TargetActor->GetActorLocation();
		ResolvedIntent.Hold.bCaptureCurrentPosition = false;
		ResolvedIntent.Hold.TargetActor = nullptr;
	}
	if (IsValid(SourceIntent.Orbit.CenterActor))
	{
		ResolvedIntent.Orbit.CenterCm = SourceIntent.Orbit.CenterActor->GetActorLocation();
		ResolvedIntent.Orbit.CenterActor = nullptr;
	}
	if (IsValid(SourceIntent.Heading.TargetActor))
	{
		ResolvedIntent.Heading.TargetPositionCm = SourceIntent.Heading.TargetActor->GetActorLocation();
		ResolvedIntent.Heading.TargetActor = nullptr;
	}
	const bool bResolvedTargetMoved =
		!Previous.Hold.PositionCm.Equals(ResolvedIntent.Hold.PositionCm, 0.01f)
		|| !Previous.Orbit.CenterCm.Equals(ResolvedIntent.Orbit.CenterCm, 0.01f)
		|| !Previous.Heading.TargetPositionCm.Equals(
			ResolvedIntent.Heading.TargetPositionCm, 0.01f);
	if (ActiveHandle.IsValid() && bResolvedTargetMoved)
	{
		++IntentRevision;
	}
}

void UAutopilotComponent::Finish(
	EAircraftMovementIntentStatus Status, EAircraftMovementFailureReason FailureReason)
{
	CurrentResult.Status = Status;
	CurrentResult.FailureReason = FailureReason;
	CurrentResult.ElapsedSeconds = ElapsedSeconds;
	const FAircraftMovementIntentResult FinishedResult = CurrentResult;
	ActiveHandle = {};
	DiagnosticLogAccumulatorSeconds = 0.0f;
	++IntentRevision;
	OnMovementIntentChanged.Broadcast(FinishedResult);
}

void UAutopilotComponent::ClearAutomaticContinuation()
{
	AutomaticContinuationIntent = {};
	AutomaticContinuationHandle = {};
}

void UAutopilotComponent::BeginPassThroughContinuation(
	const FVector& ExitVelocityCmPerSec,
	const FAircraftMovementIntentHandle SourceHandle)
{
	AutomaticContinuationIntent = {};
	AutomaticContinuationIntent.Type = EAircraftMovementIntentType::Velocity;
	AutomaticContinuationIntent.Velocity.VelocityCmPerSec = ExitVelocityCmPerSec;
	AutomaticContinuationIntent.Velocity.Frame = EAircraftVelocityFrame::World;
	AutomaticContinuationIntent.Limits = ResolvedIntent.Limits;
	AutomaticContinuationIntent.bHasRequestedMotionLimits =
		ResolvedIntent.bHasRequestedMotionLimits;
	AutomaticContinuationIntent.Heading = ResolvedIntent.Heading;
	if (AutomaticContinuationIntent.Heading.Mode == EAircraftHeadingMode::FaceTarget)
	{
		AutomaticContinuationIntent.Heading.Mode = EAircraftHeadingMode::FaceVelocity;
		AutomaticContinuationIntent.Heading.TargetActor = nullptr;
	}
	AutomaticContinuationHandle = SourceHandle;
	++IntentRevision;
}

void UAutopilotComponent::BeginTerminalHoldContinuation(
	const FVector& PositionCm,
	const float FixedYawDegrees,
	const FAircraftMovementIntentHandle SourceHandle)
{
	AutomaticContinuationIntent = {};
	AutomaticContinuationIntent.Type = EAircraftMovementIntentType::Hold;
	AutomaticContinuationIntent.Hold.PositionCm = PositionCm;
	AutomaticContinuationIntent.Hold.bCaptureCurrentPosition = false;
	AutomaticContinuationIntent.Limits = ResolvedIntent.Limits;
	AutomaticContinuationIntent.bHasRequestedMotionLimits =
		ResolvedIntent.bHasRequestedMotionLimits;
	AutomaticContinuationIntent.Heading.Mode = EAircraftHeadingMode::FixedYaw;
	AutomaticContinuationIntent.Heading.FixedYawDegrees = FixedYawDegrees;
	AutomaticContinuationHandle = SourceHandle;
	++IntentRevision;
}

void UAutopilotComponent::UpdateCompletion(float DeltaTime)
{
	IAircraftFlightControllerInterface* Controller = GetFlightController();
	if (!Controller || !ActiveHandle.IsValid())
	{
		return;
	}
	if (ResolvedIntent.Type == EAircraftMovementIntentType::Velocity
		|| ResolvedIntent.Type == EAircraftMovementIntentType::Orbit)
	{
		return;
	}

	FAircraftFlightKinematicState State;
	if (!Controller->GetAircraftFlightKinematicState(State))
	{
		return;
	}
	FVector Target = ResolvedIntent.Hold.PositionCm;
	if (ResolvedIntent.Type == EAircraftMovementIntentType::Route)
	{
		Target = ResolvedIntent.Route.PointsCm.Last();
	}
	else if (ResolvedIntent.Type == EAircraftMovementIntentType::TimedTrajectory)
	{
		Target = ResolvedIntent.TimedTrajectory.Samples.Last().PositionCm;
	}
	const FVector Error = Target - State.PositionCm;
	const float DistanceToTargetCm = static_cast<float>(Error.Size());
	FAircraftTrajectoryReference Reference;
	const bool bHasReference = Controller->GetAircraftTrajectoryReference(Reference)
		&& Reference.bValid;
	if (InitialDistanceToTargetCm < 0.0f)
	{
		InitialDistanceToTargetCm = FMath::Max(DistanceToTargetCm, 1.0f);
	}
	if (ResolvedIntent.Type == EAircraftMovementIntentType::Route)
	{
		CurrentResult.Progress = bHasReference ? Reference.PathProgress : 0.0f;
	}
	else if (ResolvedIntent.Type == EAircraftMovementIntentType::TimedTrajectory)
	{
		CurrentResult.Progress = FMath::Clamp(ElapsedSeconds / FMath::Max(
			ResolvedIntent.TimedTrajectory.Samples.Last().TimeSeconds, UE_SMALL_NUMBER), 0.0f, 1.0f);
	}
	else
	{
		CurrentResult.Progress = FMath::Clamp(
			1.0f - DistanceToTargetCm / InitialDistanceToTargetCm, 0.0f, 1.0f);
	}
	// 完成判据加空间约束：Progress 由时间/投影驱动，避障绕行期间可能走到 1.0
	// 而机体仍在目标远处——必须同时处于位置容差内才算路径完成。
	const bool bPathComplete = ResolvedIntent.Type != EAircraftMovementIntentType::Route
		|| (CurrentResult.Progress >= 0.999f
			&& DistanceToTargetCm <= FMath::Max(
				ResolvedIntent.Completion.HorizontalToleranceCm,
				ResolvedIntent.Completion.VerticalToleranceCm) * 2.0f);
	const bool bWithinPosition = FVector2D(Error.X, Error.Y).Size()
		<= ResolvedIntent.Completion.HorizontalToleranceCm
		&& FMath::Abs(Error.Z) <= ResolvedIntent.Completion.VerticalToleranceCm;
	if (ResolvedIntent.Completion.ArrivalMode == EAircraftArrivalMode::PassThrough)
	{
		if (bWithinPosition && bPathComplete)
		{
			const FAircraftMovementIntentHandle CompletedHandle = ActiveHandle;
			const FVector ExitVelocityCmPerSec = bHasReference
				? Reference.VelocityCmPerSec
				: State.VelocityCmPerSec;
			CurrentResult.Progress = 1.0f;
			Finish(EAircraftMovementIntentStatus::Succeeded,
				EAircraftMovementFailureReason::None);
			if (bActive && !ActiveHandle.IsValid())
			{
				BeginPassThroughContinuation(ExitVelocityCmPerSec, CompletedHandle);
			}
		}
		return;
	}
	const bool bWithinSpeed = State.VelocityCmPerSec.Size()
		<= FMath::Max(ResolvedIntent.Completion.TerminalSpeedCmPerSec,
			ResolvedIntent.Completion.SpeedToleranceCmPerSec);
	FVector HeadingVelocity = State.VelocityCmPerSec;
	if (ResolvedIntent.Type == EAircraftMovementIntentType::Route
		&& ResolvedIntent.Route.PointsCm.Num() >= 2)
	{
		HeadingVelocity = ResolvedIntent.Route.PointsCm.Last()
			- ResolvedIntent.Route.PointsCm[ResolvedIntent.Route.PointsCm.Num() - 2];
	}
	else if (ResolvedIntent.Type == EAircraftMovementIntentType::TimedTrajectory)
	{
		HeadingVelocity = ResolvedIntent.TimedTrajectory.Samples.Last().VelocityCmPerSec;
	}
	const float DesiredYaw = FAircraftMotionPlan::ResolveYaw(
		ResolvedIntent.Heading, State.PositionCm, HeadingVelocity, State.AttitudeDegrees.Yaw);
	const bool bWithinYaw = FMath::Abs(FMath::FindDeltaAngleDegrees(
		State.AttitudeDegrees.Yaw, DesiredYaw)) <= ResolvedIntent.Completion.YawToleranceDegrees;
	StableTimeSeconds = bPathComplete && bWithinPosition && bWithinSpeed && bWithinYaw
		? StableTimeSeconds + DeltaTime : 0.0f;
	if (StableTimeSeconds >= ResolvedIntent.Completion.StableTimeSeconds)
	{
		const FAircraftMovementIntentHandle CompletedHandle = ActiveHandle;
		CurrentResult.Progress = 1.0f;
		Finish(EAircraftMovementIntentStatus::Succeeded,
			EAircraftMovementFailureReason::None);
		if (bActive && !ActiveHandle.IsValid())
		{
			BeginTerminalHoldContinuation(Target, DesiredYaw, CompletedHandle);
		}
	}
}

void UAutopilotComponent::TickComponent(
	float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction)
{
	Super::TickComponent(DeltaTime, TickType, ThisTickFunction);
	ResolveFlightController();
	if (!ActiveHandle.IsValid())
	{
		return;
	}
	if (!AcquireFlightControl())
	{
		Finish(EAircraftMovementIntentStatus::Failed,
			EAircraftMovementFailureReason::FlightControllerUnavailable);
		return;
	}
	ResolveActorTargets();
	ElapsedSeconds += DeltaTime;
	CurrentResult.ElapsedSeconds = ElapsedSeconds;
	IAircraftFlightControllerInterface* const Controller = GetFlightController();
	FAircraftAutopilotDiagnostics Diagnostics;
	const bool bMatchingDiagnostics = Controller
		&& Controller->GetAircraftAutopilotDiagnostics(Diagnostics)
		&& Diagnostics.ActiveIntentId == ActiveHandle.Id
		&& Diagnostics.IntentRevision == IntentRevision;
	if (bMatchingDiagnostics)
	{
		CurrentResult.PathTrackingState = Diagnostics.PathTrackingState;
		CurrentResult.ContourErrorCm = Diagnostics.ContourErrorCm;
		CurrentResult.CorridorViolationCm = Diagnostics.CorridorViolationCm;
		if (!Diagnostics.bPlanValid)
		{
			Finish(EAircraftMovementIntentStatus::Failed,
				EAircraftMovementFailureReason::PlanningFailed);
			return;
		}
		if (Diagnostics.bSolverFailed)
		{
			Finish(EAircraftMovementIntentStatus::Failed,
				EAircraftMovementFailureReason::SolverFailed);
			return;
		}
		CurrentResult.Status = Diagnostics.bReferenceFresh
			? EAircraftMovementIntentStatus::Executing
			: EAircraftMovementIntentStatus::Planning;
	}
	else
	{
		CurrentResult.Status = EAircraftMovementIntentStatus::Planning;
	}
	const FAircraftDiagnosticLogSelection LogSelection =
		UE::AircraftLab::Diagnostics::GetAircraftDiagnosticLogSelection();
	if (LogSelection.IsEnabled(EAircraftDiagnosticLogChannel::Autopilot))
	{
		DiagnosticLogAccumulatorSeconds += DeltaTime;
		if (LogSelection.IntervalSeconds <= UE_SMALL_NUMBER
			|| DiagnosticLogAccumulatorSeconds + UE_SMALL_NUMBER >= LogSelection.IntervalSeconds)
		{
			DiagnosticLogAccumulatorSeconds = 0.0f;
			FAircraftTrajectoryReference Reference;
			const bool bHasReference = Controller
				&& Controller->GetAircraftTrajectoryReference(Reference) && Reference.bValid;
			UE_LOG(LogAircraft, Log,
				TEXT("[Aircraft.Autopilot] Owner=%s Intent=%lld Revision=%llu Status=%s Tracking=%s Ref=%d PathProgress=%.3f RouteProgress=%.3f Contour=%.1fcm Lag=%.1fcm Corridor=%.1fcm Predicted=%.1fcm Solve=%.3fms"),
				*GetNameSafe(GetOwner()), ActiveHandle.Id, IntentRevision,
				*UEnum::GetDisplayValueAsText(CurrentResult.Status).ToString(),
				*UEnum::GetDisplayValueAsText(CurrentResult.PathTrackingState).ToString(),
				bHasReference ? 1 : 0, Reference.PathProgress, Reference.RouteProgress,
				Diagnostics.ContourErrorCm, Diagnostics.LagErrorCm,
				Diagnostics.CorridorViolationCm, Diagnostics.PredictedCorridorViolationCm,
				Diagnostics.LastSolveMilliseconds);
		}
	}
	if (ResolvedIntent.TimeoutSeconds > 0.0f && ElapsedSeconds >= ResolvedIntent.TimeoutSeconds)
	{
		Finish(EAircraftMovementIntentStatus::Failed,
			EAircraftMovementFailureReason::Timeout);
		return;
	}
	UpdateCompletion(DeltaTime);
}

bool UAutopilotComponent::GetAircraftMovementIntent(
	FAircraftMovementIntent& OutIntent, FAircraftMovementIntentHandle& OutHandle,
	uint64& OutRevision) const
{
	if (!IsAircraftMovementIntentActive())
	{
		return false;
	}
	const bool bHasExternalIntent = ActiveHandle.IsValid();
	OutIntent = bHasExternalIntent ? ResolvedIntent : AutomaticContinuationIntent;
	OutHandle = bHasExternalIntent ? ActiveHandle : AutomaticContinuationHandle;
	OutRevision = IntentRevision;
	return true;
}

void UAutopilotComponent::AppendDebugSnapshot(const FAircraftDebugCaptureRequest& Request,
	FAircraftDebugFrameSnapshot& Snapshot) const
{
	if (!Request.Requires(EAircraftDebugPayload::AutopilotCore)
		|| !IsAircraftMovementIntentActive())
	{
		return;
	}
	IAircraftFlightControllerInterface* const Controller = GetFlightController();
	if (!Controller || !Controller->GetAircraftFlightKinematicState(Snapshot.AutopilotState))
	{
		return;
	}
	Snapshot.AvailablePayloads |= EAircraftDebugPayload::AutopilotCore;
	const FAircraftMovementIntent& Intent = ActiveHandle.IsValid()
		? ResolvedIntent : AutomaticContinuationIntent;
	Snapshot.AutopilotIntentType = Intent.Type;
	Controller->GetAircraftTrajectoryReference(Snapshot.AutopilotReference);
	Controller->GetAircraftAutopilotDiagnostics(Snapshot.AutopilotDiagnostics);
	if (Request.Requires(EAircraftDebugPayload::AutopilotPlan))
	{
		Controller->GetAircraftMotionPlan(Snapshot.AutopilotPlanSamples,
			Snapshot.AutopilotPlanDurationSeconds, Snapshot.AutopilotPlanLengthCm,
			Snapshot.AutopilotPlanRevision);
		if (Intent.Type == EAircraftMovementIntentType::Route
			&& Snapshot.AutopilotPlanSamples.IsEmpty())
		{
			Snapshot.AutopilotRoutePointsCm = Intent.Route.PointsCm;
			Snapshot.bAutopilotRouteClosed = Intent.Route.bClosed;
		}
		Snapshot.AvailablePayloads |= EAircraftDebugPayload::AutopilotPlan;
	}
	if (Request.Requires(EAircraftDebugPayload::AutopilotCorridor))
	{
		if (Intent.Type == EAircraftMovementIntentType::Route)
		{
			Snapshot.AutopilotCorridor = Intent.Route.Corridor;
			for (int32 Index = 1; Index < Intent.Route.PointsCm.Num(); ++Index)
			{
				Snapshot.AutopilotRouteLengthCm += FVector::Distance(
					Intent.Route.PointsCm[Index - 1], Intent.Route.PointsCm[Index]);
			}
			if (Intent.Route.bClosed && Intent.Route.PointsCm.Num() > 2)
			{
				Snapshot.AutopilotRouteLengthCm += FVector::Distance(
					Intent.Route.PointsCm.Last(), Intent.Route.PointsCm[0]);
			}
		}
		Snapshot.AvailablePayloads |= EAircraftDebugPayload::AutopilotCorridor;
	}
}

bool UAutopilotComponent::IsAircraftMovementIntentActive() const
{
	return bActive && (ActiveHandle.IsValid() || AutomaticContinuationHandle.IsValid());
}

void UAutopilotComponent::OnAircraftMovementIntentInterrupted(
	const FAircraftMovementIntentHandle Handle,
	const EAircraftMovementFailureReason Reason)
{
	if (Handle == ActiveHandle)
	{
		ClearAutomaticContinuation();
		Finish(EAircraftMovementIntentStatus::Interrupted, Reason);
		SourceIntent = {};
		ResolvedIntent = {};
		StableTimeSeconds = 0.0f;
		InitialDistanceToTargetCm = -1.0f;
	}
	else if (Handle == AutomaticContinuationHandle)
	{
		ClearAutomaticContinuation();
		SourceIntent = {};
		ResolvedIntent = {};
		StableTimeSeconds = 0.0f;
		InitialDistanceToTargetCm = -1.0f;
		CurrentResult.Handle = Handle;
		CurrentResult.Status = EAircraftMovementIntentStatus::Interrupted;
		CurrentResult.FailureReason = Reason;
		++IntentRevision;
		OnMovementIntentChanged.Broadcast(CurrentResult);
	}
}

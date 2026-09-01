#include "AircraftAutopilot/AutopilotComponent.h"

#include "AircraftAutopilot/AircraftMotionPlan.h"
#include "AircraftDiagnostics/AircraftDebugRuntime.h"
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
	PassThroughContinuationHandle = {};
	AcquireFlightControl();
	SourceIntent = Intent;
	ResolvedIntent = Intent;
	ActiveHandle.Id = NextIntentId++;
	++IntentRevision;
	ElapsedSeconds = 0.0f;
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
	if (Handle != ActiveHandle)
	{
		return false;
	}
	Finish(EAircraftMovementIntentStatus::Cancelled,
		EAircraftMovementFailureReason::CancelledByCaller);
	return true;
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
		PassThroughContinuationHandle = {};
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
	++IntentRevision;
	OnMovementIntentChanged.Broadcast(FinishedResult);
}

void UAutopilotComponent::BeginPassThroughContinuation(const FVector& ExitVelocityCmPerSec)
{
	PassThroughContinuationIntent = {};
	PassThroughContinuationIntent.Type = EAircraftMovementIntentType::Velocity;
	PassThroughContinuationIntent.Velocity.VelocityCmPerSec = ExitVelocityCmPerSec;
	PassThroughContinuationIntent.Velocity.Frame = EAircraftVelocityFrame::World;
	PassThroughContinuationIntent.Limits = ResolvedIntent.Limits;
	PassThroughContinuationIntent.Heading = ResolvedIntent.Heading;
	if (PassThroughContinuationIntent.Heading.Mode == EAircraftHeadingMode::FaceTarget)
	{
		PassThroughContinuationIntent.Heading.Mode = EAircraftHeadingMode::FaceVelocity;
		PassThroughContinuationIntent.Heading.TargetActor = nullptr;
	}
	PassThroughContinuationHandle.Id = TNumericLimits<int64>::Max() - 1;
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
	const bool bPathComplete = ResolvedIntent.Type != EAircraftMovementIntentType::Route
		|| CurrentResult.Progress >= 0.999f;
	const bool bWithinPosition = FVector2D(Error.X, Error.Y).Size()
		<= ResolvedIntent.Completion.HorizontalToleranceCm
		&& FMath::Abs(Error.Z) <= ResolvedIntent.Completion.VerticalToleranceCm;
	if (ResolvedIntent.Completion.ArrivalMode == EAircraftArrivalMode::PassThrough)
	{
		if (bWithinPosition && bPathComplete)
		{
			const FVector ExitVelocityCmPerSec = bHasReference
				? Reference.VelocityCmPerSec
				: State.VelocityCmPerSec;
			CurrentResult.Progress = 1.0f;
			Finish(EAircraftMovementIntentStatus::Succeeded,
				EAircraftMovementFailureReason::None);
			if (bActive && !ActiveHandle.IsValid())
			{
				BeginPassThroughContinuation(ExitVelocityCmPerSec);
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
		CurrentResult.Progress = 1.0f;
		Finish(EAircraftMovementIntentStatus::Succeeded,
			EAircraftMovementFailureReason::None);
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
	if (EnumHasAnyFlags(UE::AircraftLab::Diagnostics::GetRuntimeDebugDrawData(),
			EAircraftDebugData::Autopilot)
		&& Controller)
	{
		FAircraftDebugFrameSnapshot DebugSnapshot;
		DebugSnapshot.SubjectName = FString::Printf(TEXT("%s/%s"),
			*GetNameSafe(FlightControllerComponent->GetOwner()),
			*FlightControllerComponent->GetName());
		AppendDebugSnapshot(DebugSnapshot);
		UE::AircraftLab::Diagnostics::DrawRuntime(GetWorld(), DebugSnapshot);
	}
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
	OutIntent = bHasExternalIntent ? ResolvedIntent : PassThroughContinuationIntent;
	OutHandle = bHasExternalIntent ? ActiveHandle : PassThroughContinuationHandle;
	OutRevision = IntentRevision;
	return true;
}

void UAutopilotComponent::AppendDebugSnapshot(FAircraftDebugFrameSnapshot& Snapshot) const
{
	if (!IsAircraftMovementIntentActive())
	{
		return;
	}
	IAircraftFlightControllerInterface* const Controller = GetFlightController();
	if (!Controller || !Controller->GetAircraftFlightKinematicState(Snapshot.AutopilotState))
	{
		return;
	}
	Snapshot.AvailableData |= EAircraftDebugData::Autopilot;
	Snapshot.MovementIntent = ActiveHandle.IsValid() ? ResolvedIntent : PassThroughContinuationIntent;
	Controller->GetAircraftTrajectoryReference(Snapshot.AutopilotReference);
	Controller->GetAircraftAutopilotDiagnostics(Snapshot.AutopilotDiagnostics);
	Controller->GetAircraftMotionPlan(
		Snapshot.AutopilotPlanSamples,
		Snapshot.AutopilotPlanDurationSeconds,
		Snapshot.AutopilotPlanLengthCm,
		Snapshot.AutopilotPlanRevision);
}

bool UAutopilotComponent::IsAircraftMovementIntentActive() const
{
	return bActive && (ActiveHandle.IsValid() || PassThroughContinuationHandle.IsValid());
}

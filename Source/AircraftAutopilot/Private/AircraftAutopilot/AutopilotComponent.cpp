#include "AircraftAutopilot/AutopilotComponent.h"

#include "AircraftAutopilot/AircraftMotionPlan.h"
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
			return;
		}
	}
}

FAircraftMovementIntent UAutopilotComponent::BuildIntent(
	EAircraftMovementIntentType Type,
	const FAircraftMovementIntentSettings& Settings,
	const FAircraftCompletionPolicy* Completion)
{
	FAircraftMovementIntent Intent;
	Intent.Type = Type;
	Intent.Limits = Settings.Limits;
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
		return;
	}
	bActive = bInActive;
	if (!bActive && ActiveHandle.IsValid())
	{
		Finish(EAircraftMovementIntentStatus::Failed,
			EAircraftMovementFailureReason::AutopilotInactive);
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
	OnMovementIntentChanged.Broadcast(CurrentResult);
	ActiveHandle = {};
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
	if (InitialDistanceToTargetCm < 0.0f)
	{
		InitialDistanceToTargetCm = FMath::Max(DistanceToTargetCm, 1.0f);
	}
	CurrentResult.Progress = ResolvedIntent.Type == EAircraftMovementIntentType::TimedTrajectory
		? FMath::Clamp(ElapsedSeconds / FMath::Max(
			ResolvedIntent.TimedTrajectory.Samples.Last().TimeSeconds, UE_SMALL_NUMBER), 0.0f, 1.0f)
		: FMath::Clamp(1.0f - DistanceToTargetCm / InitialDistanceToTargetCm, 0.0f, 1.0f);
	const bool bWithinPosition = FVector2D(Error.X, Error.Y).Size()
		<= ResolvedIntent.Completion.HorizontalToleranceCm
		&& FMath::Abs(Error.Z) <= ResolvedIntent.Completion.VerticalToleranceCm;
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
	StableTimeSeconds = bWithinPosition && bWithinSpeed && bWithinYaw
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
	if (!GetFlightController())
	{
		Finish(EAircraftMovementIntentStatus::Failed,
			EAircraftMovementFailureReason::FlightControllerUnavailable);
		return;
	}
	ResolveActorTargets();
	ElapsedSeconds += DeltaTime;
	CurrentResult.ElapsedSeconds = ElapsedSeconds;
	FAircraftAutopilotDiagnostics Diagnostics;
	if (IAircraftFlightControllerInterface* Controller = GetFlightController();
		Controller && Controller->GetAircraftAutopilotDiagnostics(Diagnostics)
		&& Diagnostics.ActiveIntentId == ActiveHandle.Id
		&& Diagnostics.IntentRevision == IntentRevision)
	{
		if (!Diagnostics.bPlanValid)
		{
			Finish(EAircraftMovementIntentStatus::Failed,
				EAircraftMovementFailureReason::PlanningFailed);
			return;
		}
		if (!Diagnostics.bReferenceFresh && Diagnostics.ConsecutiveFailures > 0)
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
	OutIntent = ResolvedIntent;
	OutHandle = ActiveHandle;
	OutRevision = IntentRevision;
	return true;
}

bool UAutopilotComponent::IsAircraftMovementIntentActive() const
{
	return bActive && ActiveHandle.IsValid();
}

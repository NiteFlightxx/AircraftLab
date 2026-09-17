#include "AircraftAutopilot/AutopilotComponent.h"

#include "Engine/World.h"

#include "AircraftAutopilot/AircraftMotionPlan.h"
#include "AircraftAutopilot/AircraftAutopilotCompletion.h"
#include "AircraftDiagnostics/AircraftDebug.h"
#include "AircraftDiagnostics/AircraftDebugSettings.h"
#include "GameFramework/Actor.h"

#include UE_INLINE_GENERATED_CPP_BY_NAME(AutopilotComponent)

namespace
{
	void LogIntentConfiguration(const AActor* const Owner,
		const FAircraftMovementIntentHandle Handle, const uint64 Revision,
		const FAircraftMovementIntent& Intent)
	{
		if (!UE::AircraftLab::Diagnostics::GetAircraftDiagnosticLogSelection().IsEnabled(
			EAircraftDiagnosticLogChannel::Autopilot))
		{
			return;
		}

		UE_LOG(LogAircraft, Log,
			TEXT("[Aircraft.Autopilot.IntentConfig] Owner=%s Intent=%lld Revision=%llu Type=%s Arrival=%s Heading=%s OverrideLimits=%d Limits(Speed/Accel/Decel/Jerk)=(%.1f,%.1f,%.1f,%.1f) Completion(H/V/TH/TV/HS/VS/Yaw/Stable)=(%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%.3f) Timeout=%.3f"),
			*GetNameSafe(Owner), Handle.Id, Revision,
			*UEnum::GetDisplayValueAsText(Intent.Type).ToString(),
			*UEnum::GetDisplayValueAsText(Intent.Completion.ArrivalMode).ToString(),
			*UEnum::GetDisplayValueAsText(Intent.Heading.Mode).ToString(),
			Intent.bHasRequestedMotionLimits ? 1 : 0,
			Intent.Limits.CruiseSpeedCmPerSec,
			Intent.Limits.MaxAccelerationCmPerSecSq,
			Intent.Limits.MaxDecelerationCmPerSecSq,
			Intent.Limits.MaxJerkCmPerSecCubed,
			Intent.Completion.HorizontalToleranceCm,
			Intent.Completion.VerticalToleranceCm,
			Intent.Completion.TerminalHorizontalSpeedCmPerSec,
			Intent.Completion.TerminalVerticalSpeedCmPerSec,
			Intent.Completion.HorizontalSpeedToleranceCmPerSec,
			Intent.Completion.VerticalSpeedToleranceCmPerSec,
			Intent.Completion.YawToleranceDegrees,
			Intent.Completion.StableTimeSeconds,
			Intent.TimeoutSeconds);
	}
}

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
		const uint64 ExpectedRevisionAfterFinish = IntentRevision + 1;
		Finish(EAircraftMovementIntentStatus::Interrupted,
			EAircraftMovementFailureReason::Replaced);
		// 同步事件允许监听者提交更晚的新任务。该新任务拥有控制权，
		// 外层替换请求不得在回调返回后覆盖它。
		if (ActiveHandle.IsValid() || IntentRevision != ExpectedRevisionAfterFinish)
		{
			return {};
		}
	}
	ClearAutomaticContinuation();
	AcquireFlightControl();
	SourceIntent = Intent;
	ResolvedIntent = Intent;
	ActiveHandle.Id = NextIntentId++;
	++IntentRevision;
	ResetActorTargetBumpBaseline();
	ElapsedSeconds = 0.0f;
	DiagnosticLogAccumulatorSeconds = 0.0f;
	StableTimeSeconds = 0.0f;
	InitialDistanceToTargetCm = -1.0f;
	CurrentResult = {};
	CurrentResult.Handle = ActiveHandle;
	CurrentResult.Status = EAircraftMovementIntentStatus::Accepted;
	ResolveActorTargets();
	LogIntentConfiguration(GetOwner(), ActiveHandle, IntentRevision, ResolvedIntent);
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
	ResetActorTargetBumpBaseline();
	StableTimeSeconds = 0.0f;
	InitialDistanceToTargetCm = -1.0f;
	CurrentResult.Status = EAircraftMovementIntentStatus::Accepted;
	ResolveActorTargets();
	LogIntentConfiguration(GetOwner(), ActiveHandle, IntentRevision, ResolvedIntent);
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

bool UAutopilotComponent::ShouldBumpRevisionForActorTargetMove(
	const bool bHasBaseline, const double MaxAnchorShiftCm,
	const double SecondsSinceLastBump,
	const double ReplanDistanceCm, const double ReplanIntervalSeconds)
{
	if (!bHasBaseline)
	{
		return false;
	}
	const double Distance = FMath::Max(ReplanDistanceCm, 1.0);
	if (MaxAnchorShiftCm <= Distance)
	{
		return false;
	}
	// 超过 4× 滞回距离视为快速目标/瞬移，立即更新不受限频。
	if (MaxAnchorShiftCm > 4.0 * Distance)
	{
		return true;
	}
	return SecondsSinceLastBump >= FMath::Max(ReplanIntervalSeconds, 0.0);
}

void UAutopilotComponent::ResolveActorTargets()
{
	const FAircraftMovementIntent Previous = ResolvedIntent;
	ResolvedIntent = SourceIntent;
	const bool bHasHoldActor = SourceIntent.Hold.TargetActor.IsValid();
	const bool bHasOrbitActor = SourceIntent.Orbit.CenterActor.IsValid();
	const bool bHasHeadingActor = SourceIntent.Heading.TargetActor.IsValid();
	const bool bHadHoldActor = !SourceIntent.Hold.TargetActor.IsExplicitlyNull();
	const bool bHadOrbitActor = !SourceIntent.Orbit.CenterActor.IsExplicitlyNull();
	const bool bHadHeadingActor = !SourceIntent.Heading.TargetActor.IsExplicitlyNull();
	const bool bHasActorTarget = bHasHoldActor || bHasOrbitActor || bHasHeadingActor;
	if (bHasHoldActor)
	{
		ResolvedIntent.Hold.PositionCm = SourceIntent.Hold.TargetActor->GetActorLocation();
		ResolvedIntent.Hold.bCaptureCurrentPosition = false;
	}
	else if (bHadHoldActor)
	{
		ResolvedIntent.Hold.PositionCm = Previous.Hold.PositionCm;
		ResolvedIntent.Hold.bCaptureCurrentPosition = false;
	}
	if (bHasOrbitActor)
	{
		ResolvedIntent.Orbit.CenterCm = SourceIntent.Orbit.CenterActor->GetActorLocation();
	}
	else if (bHadOrbitActor)
	{
		ResolvedIntent.Orbit.CenterCm = Previous.Orbit.CenterCm;
	}
	if (bHasHeadingActor)
	{
		ResolvedIntent.Heading.TargetPositionCm = SourceIntent.Heading.TargetActor->GetActorLocation();
	}
	else if (bHadHeadingActor)
	{
		ResolvedIntent.Heading.TargetPositionCm = Previous.Heading.TargetPositionCm;
	}
	// Actor references belong to the Game Thread source intent only. The resolved
	// snapshot crossing into the simulation thread is always pointer-free.
	ResolvedIntent.Hold.TargetActor.Reset();
	ResolvedIntent.Orbit.CenterActor.Reset();
	ResolvedIntent.Heading.TargetActor.Reset();
	const UWorld* const World = GetWorld();
	const double CurrentTimeSeconds = World ? World->GetTimeSeconds() : 0.0;
	if (ActiveHandle.IsValid() && bHasActorTarget && !bHasActorTargetBumpBaseline)
	{
		// 第一次解析只把真实 Actor 锚点设为累计位移的基线。Submit/Update
		// 已经为调用方的意图修改递增过 revision，此处不能制造第二次重规划。
		HoldPositionAtLastBumpCm = ResolvedIntent.Hold.PositionCm;
		OrbitCenterAtLastBumpCm = ResolvedIntent.Orbit.CenterCm;
		HeadingTargetAtLastBumpCm = ResolvedIntent.Heading.TargetPositionCm;
		LastActorTargetBumpTimeSeconds = CurrentTimeSeconds;
		bHasActorTargetBumpBaseline = true;
		return;
	}
	// 滞回基准：与上次 bump 时的锚点比较，而不是与上一帧比较——
	// 上一帧差分（旧实现 0.01cm 阈值）使任何移动目标每帧都触发全量计划重建。
	const FVector& ReferenceHold =
		bHasActorTargetBumpBaseline ? HoldPositionAtLastBumpCm : Previous.Hold.PositionCm;
	const FVector& ReferenceOrbit =
		bHasActorTargetBumpBaseline ? OrbitCenterAtLastBumpCm : Previous.Orbit.CenterCm;
	const FVector& ReferenceHeading =
		bHasActorTargetBumpBaseline ? HeadingTargetAtLastBumpCm : Previous.Heading.TargetPositionCm;
	const double MaxAnchorShiftCm = FMath::Max(FMath::Max(
		FVector::Dist(ReferenceHold, ResolvedIntent.Hold.PositionCm),
		FVector::Dist(ReferenceOrbit, ResolvedIntent.Orbit.CenterCm)),
		FVector::Dist(ReferenceHeading, ResolvedIntent.Heading.TargetPositionCm));
	const double SecondsSinceLastBump = bHasActorTargetBumpBaseline
		? FMath::Max(CurrentTimeSeconds - LastActorTargetBumpTimeSeconds, 0.0)
		: TNumericLimits<double>::Max();
	if (ActiveHandle.IsValid()
		&& bHasActorTarget
		&& ShouldBumpRevisionForActorTargetMove(bHasActorTargetBumpBaseline,
			MaxAnchorShiftCm, SecondsSinceLastBump,
			ActorTargetReplanDistanceCm, ActorTargetReplanIntervalSeconds))
	{
		HoldPositionAtLastBumpCm = ResolvedIntent.Hold.PositionCm;
		OrbitCenterAtLastBumpCm = ResolvedIntent.Orbit.CenterCm;
		HeadingTargetAtLastBumpCm = ResolvedIntent.Heading.TargetPositionCm;
		LastActorTargetBumpTimeSeconds = CurrentTimeSeconds;
		bHasActorTargetBumpBaseline = true;
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
	SourceIntent = {};
	ResolvedIntent = {};
	StableTimeSeconds = 0.0f;
	InitialDistanceToTargetCm = -1.0f;
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
	const FAircraftMovementIntent& CompletedIntent,
	const FVector& ExitVelocityCmPerSec,
	const FAircraftMovementIntentHandle SourceHandle)
{
	AutomaticContinuationIntent = {};
	AutomaticContinuationIntent.Type = EAircraftMovementIntentType::Velocity;
	AutomaticContinuationIntent.Velocity.VelocityCmPerSec = ExitVelocityCmPerSec;
	AutomaticContinuationIntent.Velocity.Frame = EAircraftVelocityFrame::World;
	AutomaticContinuationIntent.Limits = CompletedIntent.Limits;
	AutomaticContinuationIntent.bHasRequestedMotionLimits =
		CompletedIntent.bHasRequestedMotionLimits;
	AutomaticContinuationIntent.Heading = CompletedIntent.Heading;
	if (AutomaticContinuationIntent.Heading.Mode == EAircraftHeadingMode::FaceTarget)
	{
		AutomaticContinuationIntent.Heading.Mode = EAircraftHeadingMode::FaceVelocity;
		AutomaticContinuationIntent.Heading.TargetActor = nullptr;
	}
	AutomaticContinuationHandle = SourceHandle;
	++IntentRevision;
}

void UAutopilotComponent::BeginTerminalContinuation(
	const FVector& PositionCm,
	const float FixedYawDegrees,
	const FAircraftMovementIntent& CompletedIntent,
	const FAircraftMovementIntentHandle SourceHandle)
{
	AutomaticContinuationIntent =
		UE::AircraftLab::Autopilot::Private::BuildTerminalContinuationIntent(
			CompletedIntent, PositionCm, FixedYawDegrees);
	AutomaticContinuationHandle = SourceHandle;
	++IntentRevision;
	if (UE::AircraftLab::Diagnostics::GetAircraftDiagnosticLogSelection().IsEnabled(
		EAircraftDiagnosticLogChannel::Autopilot))
	{
		UE_LOG(LogAircraft, Log,
			TEXT("[Aircraft.Autopilot.Continuation] Owner=%s Intent=%lld Revision=%llu CompletedType=%s ContinuationType=%s ReusePlan=%d PositionCm=%s Yaw=%.2f"),
			*GetNameSafe(GetOwner()), SourceHandle.Id, IntentRevision,
			*UEnum::GetDisplayValueAsText(CompletedIntent.Type).ToString(),
			*UEnum::GetDisplayValueAsText(AutomaticContinuationIntent.Type).ToString(),
			AutomaticContinuationIntent.Type == CompletedIntent.Type ? 1 : 0,
			*PositionCm.ToCompactString(), FixedYawDegrees);
	}
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
	FAircraftTrajectoryReference Reference;
	const bool bHasReference = Controller->GetAircraftTrajectoryReference(Reference)
		&& Reference.bValid;
	FVector Target;
	if (!UE::AircraftLab::Autopilot::Private::ResolveCompletionTarget(
		ResolvedIntent, bHasReference, Reference, Target))
	{
		return;
	}
	const FVector Error = Target - State.PositionCm;
	const float DistanceToTargetCm = static_cast<float>(Error.Size());
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
		CurrentResult.Progress = bHasReference ? Reference.PathProgress : 0.0f;
	}
	else
	{
		CurrentResult.Progress = FMath::Clamp(
			1.0f - DistanceToTargetCm / InitialDistanceToTargetCm, 0.0f, 1.0f);
	}
	// 完成判据加空间约束：Progress 由时间/投影驱动，避障绕行期间可能走到 1.0
	// 而机体仍在目标远处——必须同时处于位置容差内才算路径完成。
	const bool bPathComplete = UE::AircraftLab::Autopilot::Private::IsPlanComplete(
		ResolvedIntent.Type, bHasReference, CurrentResult.Progress);
	const bool bWithinPosition = FVector2D(Error.X, Error.Y).Size()
		<= ResolvedIntent.Completion.HorizontalToleranceCm
		&& FMath::Abs(Error.Z) <= ResolvedIntent.Completion.VerticalToleranceCm;
	if (ResolvedIntent.Completion.ArrivalMode == EAircraftArrivalMode::PassThrough)
	{
		if (bWithinPosition && bPathComplete)
		{
			const FAircraftMovementIntentHandle CompletedHandle = ActiveHandle;
			const FAircraftMovementIntent CompletedIntent = ResolvedIntent;
			const FVector ExitVelocityCmPerSec = bHasReference
				? Reference.VelocityCmPerSec
				: State.VelocityCmPerSec;
			CurrentResult.Progress = 1.0f;
			Finish(EAircraftMovementIntentStatus::Succeeded,
				EAircraftMovementFailureReason::None);
			if (bActive && !ActiveHandle.IsValid())
			{
				BeginPassThroughContinuation(
					CompletedIntent, ExitVelocityCmPerSec, CompletedHandle);
			}
		}
		return;
	}
	const bool bWithinHorizontalSpeed = FVector2D(
		State.VelocityCmPerSec.X, State.VelocityCmPerSec.Y).Size()
		<= FMath::Max(ResolvedIntent.Completion.TerminalHorizontalSpeedCmPerSec,
			ResolvedIntent.Completion.HorizontalSpeedToleranceCmPerSec);
	const bool bWithinVerticalSpeed = FMath::Abs(State.VelocityCmPerSec.Z)
		<= FMath::Max(ResolvedIntent.Completion.TerminalVerticalSpeedCmPerSec,
			ResolvedIntent.Completion.VerticalSpeedToleranceCmPerSec);
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
	const bool bArrivalConditionsSatisfied = bPathComplete && bWithinPosition
		&& bWithinHorizontalSpeed && bWithinVerticalSpeed && bWithinYaw;
	StableTimeSeconds = bArrivalConditionsSatisfied
		? StableTimeSeconds + DeltaTime : 0.0f;
	if (UE::AircraftLab::Autopilot::Private::HasStableCompletion(
		bArrivalConditionsSatisfied, StableTimeSeconds,
		ResolvedIntent.Completion.StableTimeSeconds))
	{
		const FAircraftMovementIntentHandle CompletedHandle = ActiveHandle;
		const FAircraftMovementIntent CompletedIntent = ResolvedIntent;
		CurrentResult.Progress = 1.0f;
		Finish(EAircraftMovementIntentStatus::Succeeded,
			EAircraftMovementFailureReason::None);
		if (bActive && !ActiveHandle.IsValid())
		{
			BeginTerminalContinuation(
				Target, DesiredYaw, CompletedIntent, CompletedHandle);
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
			FAircraftFlightKinematicState FlightState;
			const bool bHasFlightState = Controller
				&& Controller->GetAircraftFlightKinematicState(FlightState);
			FVector CompletionTargetCm = FVector::ZeroVector;
			const bool bHasCompletionTarget =
				UE::AircraftLab::Autopilot::Private::ResolveCompletionTarget(
					ResolvedIntent, bHasReference, Reference, CompletionTargetCm);
			UE_LOG(LogAircraft, Log,
				TEXT("[Aircraft.Autopilot] Owner=%s Intent=%lld Revision=%llu Status=%s Tracking=%s Ref=%d State=%d Target=%d PositionCm=%s TargetCm=%s RefPositionCm=%s VelocityCmPerSec=%s PathProgress=%.3f RouteProgress=%.3f ProgressScale=%.3f SignedTerminalCm=%.1f Contour=%.1fcm Lag=%.1fcm Corridor=%.1fcm Predicted=%.1fcm Solve=%.3fms"),
				*GetNameSafe(GetOwner()), ActiveHandle.Id, IntentRevision,
				*UEnum::GetDisplayValueAsText(CurrentResult.Status).ToString(),
				*UEnum::GetDisplayValueAsText(CurrentResult.PathTrackingState).ToString(),
				bHasReference ? 1 : 0, bHasFlightState ? 1 : 0,
				bHasCompletionTarget ? 1 : 0,
				*FlightState.PositionCm.ToCompactString(),
				*CompletionTargetCm.ToCompactString(),
				*Reference.PositionCm.ToCompactString(),
				*FlightState.VelocityCmPerSec.ToCompactString(),
				Reference.PathProgress, Reference.RouteProgress,
				Diagnostics.ProgressScale, Diagnostics.SignedTerminalDistanceCm,
				Diagnostics.ContourErrorCm, Diagnostics.LagErrorCm,
				Diagnostics.CorridorViolationCm, Diagnostics.PredictedCorridorViolationCm,
				Diagnostics.LastSolveMilliseconds);
			UE_LOG(LogAircraft, Log,
				TEXT("[Aircraft.Autopilot.Reference] Owner=%s Intent=%lld MotionPlanA=%s MpccCorrection=%s TerminalBrake=%s CommandA=%s NominalVel=%s NominalA=%s FinalVel=%s FinalA=%s TerminalBrakeActive=%d GuidanceApplied=%d GuidanceBraking=%d GuidanceAlpha=%.3f"),
				*GetNameSafe(GetOwner()), ActiveHandle.Id,
				*Diagnostics.MotionPlanAccelerationCmPerSecSq.ToCompactString(),
				*Diagnostics.MpccCorrectionCmPerSecSq.ToCompactString(),
				*Diagnostics.TerminalBrakeCorrectionCmPerSecSq.ToCompactString(),
				*Diagnostics.CommandAccelerationCmPerSecSq.ToCompactString(),
				*Diagnostics.NominalReferenceVelocityCmPerSec.ToCompactString(),
				*Diagnostics.NominalReferenceControlAccelerationCmPerSecSq.ToCompactString(),
				*Diagnostics.FinalReferenceVelocityCmPerSec.ToCompactString(),
				*Diagnostics.FinalReferenceControlAccelerationCmPerSecSq.ToCompactString(),
				Diagnostics.bTerminalBrakingActive ? 1 : 0,
				Diagnostics.bGuidanceApplied ? 1 : 0,
				Diagnostics.bGuidanceBraking ? 1 : 0,
				Diagnostics.GuidanceBlendAlpha);
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
	}
	else if (Handle == AutomaticContinuationHandle)
	{
		ClearAutomaticContinuation();
		CurrentResult.Handle = Handle;
		CurrentResult.Status = EAircraftMovementIntentStatus::Interrupted;
		CurrentResult.FailureReason = Reason;
		++IntentRevision;
		OnMovementIntentChanged.Broadcast(CurrentResult);
	}
}

// （经 IAircraftSimulationLODConsumer）+ 按命令类型发布驱动覆盖。

#include "AircraftRuntimeCommon/Autopilot/AutopilotComponent.h"
#include "AircraftRuntimeCommon/Autopilot/AircraftAutopilotDebugDraw.h"

#include "Animation/AnimInstance.h"
#include "Animation/AnimMontage.h"
#include "Components/SkeletalMeshComponent.h"
#include "Engine/World.h"
#include "GameFramework/Actor.h"

#include UE_INLINE_GENERATED_CPP_BY_NAME(AutopilotComponent)

DEFINE_LOG_CATEGORY_STATIC(LogAircraftAutopilot, Log, All);

namespace
{
bool AreRootMotionConstraintsFinite(const FTrajectoryMotionConstraints& Constraints)
{
	return FMath::IsFinite(Constraints.CruiseSpeedCmPerSec)
		&& FMath::IsFinite(Constraints.MaxAccelerationCmPerSecSq)
		&& FMath::IsFinite(Constraints.MaxDecelerationCmPerSecSq)
		&& FMath::IsFinite(Constraints.MaxJerkCmPerSecCubed)
		&& FMath::IsFinite(Constraints.MaxClimbRateCmPerSec)
		&& FMath::IsFinite(Constraints.MaxDescentRateCmPerSec)
		&& FMath::IsFinite(Constraints.MaxVerticalAccelerationCmPerSecSq)
		&& FMath::IsFinite(Constraints.MaxVerticalJerkCmPerSecCubed)
		&& FMath::IsFinite(Constraints.MaxYawRateDegPerSec)
		&& FMath::IsFinite(Constraints.MaxYawAccelerationDegPerSecSq)
		&& FMath::IsFinite(Constraints.MaxYawJerkDegPerSecCubed);
}

bool IsRootMotionArrivalFinite(const FAutopilotArrivalCriteria& Arrival)
{
	return FMath::IsFinite(Arrival.HorizontalToleranceCm)
		&& FMath::IsFinite(Arrival.VerticalToleranceCm)
		&& FMath::IsFinite(Arrival.SpeedToleranceCmPerSec)
		&& FMath::IsFinite(Arrival.YawToleranceDegrees)
		&& FMath::IsFinite(Arrival.StableTimeSeconds);
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
	ResolveAutopilotConfig(/*bForceRefresh=*/true);
	if (FlightController.GetInterface() && FlightControllerComponent)
	{
		FlightControllerComponent->AddTickPrerequisiteComponent(this);
		FAircraftFlightKinematicState State;
		if (FlightController->GetAircraftFlightKinematicState(State))
		{
			MotionProfile.Initialize(
				State.PositionCm,
				State.VelocityCmPerSec,
				State.AccelerationWorldCmPerSecSq,
				State.AttitudeDegrees.Yaw,
				State.AngularVelocityBodyDegreesPerSec.Z);
		}
	}
	RefreshSimulationTickEnabled();
}

void UAutopilotComponent::EndPlay(const EEndPlayReason::Type EndPlayReason)
{
	CleanupRootMotionIntent(/*bStopMontage=*/true);
	Super::EndPlay(EndPlayReason);
}

void UAutopilotComponent::ResolveFlightController()
{
	if (IsValid(FlightController.GetObject()) && FlightController.GetInterface())
	{
		return;
	}
	FlightControllerComponent = nullptr;
	FlightController = nullptr;
	if (AActor* const OwnerActor = GetOwner())
	{
		TArray<UActorComponent*> Components;
		OwnerActor->GetComponents(Components);
		for (UActorComponent* Component : Components)
		{
			if (Component && Component->Implements<UAircraftFlightControllerInterface>())
			{
				FlightControllerComponent = Component;
				FlightController.SetObject(Component);
				FlightController.SetInterface(Cast<IAircraftFlightControllerInterface>(Component));
				break;
			}
		}
	}
}

bool UAutopilotComponent::ResolveAutopilotConfig(bool bForceRefresh)
{
	if (bAutopilotConfigResolved && !bForceRefresh)
	{
		return true;
	}
	if (!FlightController.GetInterface())
	{
		return false;
	}
	bAutopilotConfigResolved = FlightController->GetAircraftAutopilotRuntimeConfig(AutopilotConfig);
	if (bAutopilotConfigResolved)
	{
		ApplyAutopilotConfig();
	}
	return bAutopilotConfigResolved;
}

void UAutopilotComponent::ApplyAutopilotConfig()
{
	SetPathFollowingStrategy(static_cast<EAircraftGuidanceStrategy>(AutopilotConfig.GuidanceStrategy));
	TurnBehavior.Configure(AutopilotConfig);
	float GravityCmPerSecSq = 980.0f;
	float HoverCollective = 0.5f;
	float VerticalAccelMpsSq = 0.0f;
	float CollectiveCommand = 0.0f;
	if (FlightController.GetInterface())
	{
		FlightController->GetAircraftAutopilotPhysicalState(
			GravityCmPerSecSq, HoverCollective, VerticalAccelMpsSq, CollectiveCommand);
	}
	HoverThrustEstimator.Configure(AutopilotConfig, HoverCollective);
	FeedForwardCalculator.SetPhysicalReference(GravityCmPerSecSq, HoverCollective);
}

void UAutopilotComponent::SetPathFollowingStrategy(EAircraftGuidanceStrategy Strategy)
{
	switch (Strategy)
	{
	case EAircraftGuidanceStrategy::VectorField:
	{
		TUniquePtr<FAircraftVectorFieldGuidance> Guidance = MakeUnique<FAircraftVectorFieldGuidance>();
		Guidance->Configure(
			AutopilotConfig.VectorFieldCrossTrackGain,
			AutopilotConfig.VectorFieldMaxCrossTrackCorrectionCm);
		PathFollowing = MoveTemp(Guidance);
		break;
	}
	case EAircraftGuidanceStrategy::Direct:
		PathFollowing.Reset();
		break;
	case EAircraftGuidanceStrategy::PurePursuit:
	default:
	{
		TUniquePtr<FAircraftPurePursuitGuidance> Guidance = MakeUnique<FAircraftPurePursuitGuidance>();
		Guidance->Configure(
			AutopilotConfig.PurePursuitLookAheadGain,
			AutopilotConfig.PurePursuitMinLookAheadCm,
			AutopilotConfig.PurePursuitMaxLookAheadCm);
		PathFollowing = MoveTemp(Guidance);
		break;
	}
	}
	if (PathFollowing.IsValid())
	{
		PathFollowing->SetTrajectory(MovementExecutor.GetTrajectoryGenerator());
	}
}

void UAutopilotComponent::ApplyIntentMotionLimits()
{
	if (!FlightController.GetInterface())
	{
		return;
	}
	const FTrajectoryMotionConstraints& Constraints = MovementExecutor.GetActiveIntent().MotionConstraints;
	float MaxSpeedCmPerSec = Constraints.CruiseSpeedCmPerSec;
	float MaxAccelerationCmPerSecSq = Constraints.MaxAccelerationCmPerSecSq;
	FlightController->GetAircraftAutopilotMotionLimits(
		Constraints.CruiseSpeedCmPerSec, MaxSpeedCmPerSec, MaxAccelerationCmPerSecSq);
	MovementExecutor.SetPhysicalMotionLimits(MaxSpeedCmPerSec, MaxAccelerationCmPerSecSq);

	// MotionProfile 软限幅 ≤ 飞控硬限幅
	FMotionProfileLimits Limits = MotionProfile.GetLimits();
	Limits.MaxHorizontalSpeedCmPerSec = MaxSpeedCmPerSec;
	Limits.MaxHorizontalAccelCmPerSecSq = MaxAccelerationCmPerSecSq;
	const FTrajectoryMotionConstraints& C = MovementExecutor.GetActiveIntent().MotionConstraints;
	Limits.MaxHorizontalJerkCmPerSecCubed = C.MaxJerkCmPerSecCubed;
	Limits.MaxClimbRateCmPerSec = C.MaxClimbRateCmPerSec;
	Limits.MaxDescentRateCmPerSec = C.MaxDescentRateCmPerSec;
	Limits.MaxVerticalAccelCmPerSecSq = C.MaxVerticalAccelerationCmPerSecSq;
	Limits.MaxVerticalJerkCmPerSecCubed = C.MaxVerticalJerkCmPerSecCubed;
	Limits.MaxYawRateDegPerSec = C.MaxYawRateDegPerSec;
	Limits.MaxYawAccelDegPerSecSq = C.MaxYawAccelerationDegPerSecSq;
	Limits.MaxYawJerkDegPerSecCubed = C.MaxYawJerkDegPerSecCubed;
	MotionProfile.SetLimits(Limits);
}

void UAutopilotComponent::SetAutopilotActive(bool bActive)
{
	if (bActive == bAutopilotActive && bActivationInitialized == bActive)
	{
		return;
	}
	if (!FlightController.GetInterface()) ResolveFlightController();
	if (!FlightController.GetInterface())
	{
		UE_LOG(LogAircraftAutopilot, Error,
			TEXT("Cannot change Autopilot state on '%s' without a FlightController."),
			*GetNameSafe(GetOwner()));
		return;
	}

	bAutopilotActive = bActive;
	if (bActive)
	{
		ResolveAutopilotConfig(/*bForceRefresh=*/true);
		if (!bFlightModeBeforeActivationCaptured)
		{
			FlightModeBeforeActivation = FlightController->ActivateAircraftAutopilotControl();
			bFlightModeBeforeActivationCaptured = true;
		}
		bActivationInitialized = true;
		FAircraftAutopilotVehicleSnapshot Snapshot;
		if (CaptureSnapshot(Snapshot))
		{
			MotionProfile.Initialize(
				Snapshot.PositionCm, Snapshot.VelocityCmPerSec, Snapshot.AccelerationCmPerSecSq,
				Snapshot.YawDegrees, 0.0f);
			MovementExecutor.EnterHold(Snapshot);
		}
	}
	else
	{
		CleanupRootMotionIntent(/*bStopMontage=*/true);
		MovementExecutor.CancelActive(EAutopilotIntentFailureReason::CancelledByCaller);
		BroadcastIntentEvents();
		if (bFlightModeBeforeActivationCaptured)
		{
			FlightController->DeactivateAircraftAutopilotControl(FlightModeBeforeActivation);
		}
		bFlightModeBeforeActivationCaptured = false;
		bActivationInitialized = false;
		MovementExecutor.GetTrajectoryGenerator()->Clear();
		InvalidateOutputs();
	}
	RefreshSimulationTickEnabled();
}

bool UAutopilotComponent::CaptureSnapshot(FAircraftAutopilotVehicleSnapshot& OutSnapshot) const
{
	if (!FlightController.GetInterface())
	{
		return false;
	}
	FAircraftFlightKinematicState State;
	if (!FlightController->GetAircraftFlightKinematicState(State))
	{
		return false;
	}
	OutSnapshot.PositionCm = State.PositionCm;
	OutSnapshot.VelocityCmPerSec = State.VelocityCmPerSec;
	OutSnapshot.AccelerationCmPerSecSq = State.AccelerationWorldCmPerSecSq;
	OutSnapshot.YawDegrees = State.AttitudeDegrees.Yaw;
	return true;
}

void UAutopilotComponent::InvalidateOutputs()
{
	CachedProfiledSetpoint = FProfiledSetpoint();
	CachedFeedForward = FFeedForward();
	CachedGuidanceCommand = FGuidanceCommand();
	CachedTurnCommand = FTurnCommand();
}

void UAutopilotComponent::TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction)
{
	Super::TickComponent(DeltaTime, TickType, ThisTickFunction);

	if (!bAutopilotActive || DeltaTime <= UE_SMALL_NUMBER)
	{
		return;
	}
	if (ActiveRootMotionHandle.IsValid())
	{
		TickRootMotionIntent(DeltaTime);
		return;
	}

	FAircraftAutopilotVehicleSnapshot Snapshot;
	if (!CaptureSnapshot(Snapshot))
	{
		InvalidateOutputs();
		return;
	}

	ApplyIntentMotionLimits();
	FTrajectoryPoint NominalSetpoint;
	if (!MovementExecutor.BuildSetpoint(Snapshot, DeltaTime, CachedProfiledSetpoint, NominalSetpoint))
	{
		InvalidateOutputs();
		BroadcastIntentEvents();
		return;
	}

	const EAutopilotMovementIntentType IntentType = MovementExecutor.GetActiveIntent().Type;
	const bool bPathIntent = IntentType == EAutopilotMovementIntentType::FollowPath
		|| IntentType == EAutopilotMovementIntentType::Orbit
		|| IntentType == EAutopilotMovementIntentType::CircleArc;
	FGuidanceCommand Guidance;
	if (static_cast<EAircraftGuidanceStrategy>(AutopilotConfig.GuidanceStrategy)
			!= EAircraftGuidanceStrategy::Direct
		&& bPathIntent && PathFollowing.IsValid()
		&& MovementExecutor.GetTrajectoryGenerator()->IsValid())
	{
		if (PathFollowing->Update(Snapshot.PositionCm, Snapshot.VelocityCmPerSec, DeltaTime, Guidance)
			&& Guidance.bValid)
		{
			NominalSetpoint.VelocityCmPerSec.X = Guidance.DesiredVelocityCmPerSec.X;
			NominalSetpoint.VelocityCmPerSec.Y = Guidance.DesiredVelocityCmPerSec.Y;
		}
	}
	CachedGuidanceCommand = Guidance;

	MovementExecutor.ApplyHeading(Snapshot, NominalSetpoint);

	CachedTurnCommand = FTurnCommand();
	if (AutopilotConfig.bEnableCoordinatedTurns)
	{
		CachedTurnCommand = TurnBehavior.Compute(
			NominalSetpoint.VelocityCmPerSec, Snapshot.VelocityCmPerSec,
			Snapshot.YawDegrees, MovementExecutor.GetActiveIntent().MotionConstraints.MaxYawRateDegPerSec,
			DeltaTime);
	}

	CachedProfiledSetpoint = MotionProfile.Update(NominalSetpoint, DeltaTime);
	UpdateHoverThrustEstimate(DeltaTime);
	CachedFeedForward = FFeedForward();
	if (CachedProfiledSetpoint.bValid)
	{
		FeedForwardCalculator.Compute(CachedProfiledSetpoint, CachedFeedForward);
	}
	MovementExecutor.UpdateCompletion(Snapshot, DeltaTime, CachedProfiledSetpoint);
	BroadcastIntentEvents();
	FAircraftAutopilotDebugDraw::Draw(
		GetWorld(), MovementExecutor.GetTrajectoryGenerator(), NominalSetpoint,
		Guidance, Snapshot.PositionCm);
}

void UAutopilotComponent::UpdateHoverThrustEstimate(float DeltaSeconds)
{
	if (!FlightController.GetInterface())
	{
		return;
	}
	float GravityCmPerSecSq = 980.0f;
	float HoverCollective = 0.5f;
	float VerticalAccelMpsSq = 0.0f;
	float CollectiveCommand = 0.0f;
	FlightController->GetAircraftAutopilotPhysicalState(
		GravityCmPerSecSq, HoverCollective, VerticalAccelMpsSq, CollectiveCommand);
	if (!AutopilotConfig.bEnableHoverThrustEstimator)
	{
		FeedForwardCalculator.SetPhysicalReference(GravityCmPerSecSq, HoverCollective);
		return;
	}
	HoverThrustEstimator.Update(
		DeltaSeconds, VerticalAccelMpsSq, CollectiveCommand, GravityCmPerSecSq * 0.01f);
	FeedForwardCalculator.SetPhysicalReference(
		GravityCmPerSecSq, HoverThrustEstimator.GetHoverThrust());
}

void UAutopilotComponent::BuildInjection(FAutopilotInjection& OutInjection) const
{
	OutInjection.PositionSetpointCm = CachedProfiledSetpoint.PositionCm;
	OutInjection.VelocitySetpointCmPerSec = CachedFeedForward.VelocityFFCmPerSec;
	OutInjection.AccelerationSetpointCmPerSecSq = CachedFeedForward.AccelFFCmPerSecSq;
	OutInjection.AltitudeSetpointCm = CachedProfiledSetpoint.PositionCm.Z;
	OutInjection.VerticalVelocitySetpointCmPerSec = CachedProfiledSetpoint.VelocityCmPerSec.Z;
	OutInjection.ThrustFeedForward = CachedFeedForward.ThrustFF;
	OutInjection.YawSetpointDegrees = CachedProfiledSetpoint.YawDegrees;
	OutInjection.YawRateSetpointDegPerSec = CachedFeedForward.YawRateFFDegPerSec
		+ CachedTurnCommand.DesiredYawRateDegPerSec;
	OutInjection.YawRateLimitDegPerSec =
		MovementExecutor.GetActiveIntent().MotionConstraints.MaxYawRateDegPerSec;
	OutInjection.TurnRollDegrees = CachedTurnCommand.DesiredRollDegrees;
	OutInjection.bValid = true;
}

bool UAutopilotComponent::GetAutopilotInjection(FAutopilotInjection& OutInjection) const
{
	const bool bRootMotionBypassesFlightController = ActiveRootMotionHandle.IsValid()
		&& ActiveRootMotionDriveMode != EAircraftSimulationDriveMode::FlightController;
	if (!bAutopilotActive || bRootMotionBypassesFlightController
		|| !CachedProfiledSetpoint.bValid)
	{
		OutInjection = FAutopilotInjection();
		return false;
	}
	BuildInjection(OutInjection);
	return OutInjection.bValid;
}

void UAutopilotComponent::BroadcastIntentEvents()
{
	TArray<FAutopilotIntentResult> Started;
	TArray<FAutopilotIntentResult> Finished;
	MovementExecutor.DrainEvents(Started, Finished);
	for (const FAutopilotIntentResult& Result : Started)
	{
		OnIntentStarted.Broadcast(Result);
	}
	for (const FAutopilotIntentResult& Result : Finished)
	{
		OnIntentFinished.Broadcast(Result);
	}
}

/* ---------------------------------------------------------------------------
 * 意图提交（类型化 API → 通用 Intent 装配）
 * ------------------------------------------------------------------------- */

void UAutopilotComponent::ApplyHeadingOptions(FAutopilotMovementIntent& Intent, const FAutopilotHeadingOptions& Heading)
{
	Intent.HeadingMode = Heading.Mode;
	Intent.FixedYawDegrees = FRotator::NormalizeAxis(Heading.FixedYawDegrees);
	Intent.DesiredYawRateDegPerSec = Heading.Mode == EAutopilotHeadingMode::FixedYaw
		? FMath::Max(Heading.YawRateDegreesPerSec, 0.0f)
		: 0.0f;
	if (Heading.Mode == EAutopilotHeadingMode::FixedYaw
		&& Intent.DesiredYawRateDegPerSec > UE_SMALL_NUMBER)
	{
		Intent.MotionConstraints.MaxYawRateDegPerSec = FMath::Min(
			Intent.MotionConstraints.MaxYawRateDegPerSec,
			Intent.DesiredYawRateDegPerSec);
	}
	Intent.bUseIndependentHeadingTarget = Heading.bUseLookAtTarget;
	Intent.HeadingTargetPositionCm = Heading.LookAtPositionCm;
	Intent.HeadingTargetActor = Heading.LookAtActor;
}

void UAutopilotComponent::ApplyFiniteOptions(FAutopilotMovementIntent& Intent, const FAutopilotFiniteCommandOptions& Options)
{
	Intent.MotionConstraints = Options.MotionConstraints;
	ApplyHeadingOptions(Intent, Options.Heading);
	Intent.ArrivalMode = Options.ArrivalMode;
	Intent.PassThroughSpeedCmPerSec = Options.PassThroughSpeedCmPerSec;
	Intent.ArrivalCriteria = Options.ArrivalCriteria;
	Intent.TimeoutSeconds = Options.TimeoutSeconds;
}

void UAutopilotComponent::ApplyContinuousConstraints(
	FAutopilotMovementIntent& Intent,
	const FContinuousMotionConstraints& Constraints,
	float CommandedHorizontalSpeedCmPerSec)
{
	FTrajectoryMotionConstraints& Out = Intent.MotionConstraints;
	Out.CruiseSpeedCmPerSec = FMath::Max(CommandedHorizontalSpeedCmPerSec, 0.0f);
	Out.MaxAccelerationCmPerSecSq = Constraints.MaxAccelerationCmPerSecSq;
	Out.MaxDecelerationCmPerSecSq = Constraints.MaxAccelerationCmPerSecSq;
	Out.MaxJerkCmPerSecCubed = Constraints.MaxJerkCmPerSecCubed;
	Out.MaxClimbRateCmPerSec = Constraints.MaxClimbRateCmPerSec;
	Out.MaxDescentRateCmPerSec = Constraints.MaxDescentRateCmPerSec;
	Out.MaxVerticalAccelerationCmPerSecSq = Constraints.MaxVerticalAccelerationCmPerSecSq;
	Out.MaxVerticalJerkCmPerSecCubed = Constraints.MaxVerticalJerkCmPerSecCubed;
	Out.MaxYawRateDegPerSec = Constraints.MaxYawRateDegPerSec;
	Out.MaxYawAccelerationDegPerSecSq = Constraints.MaxYawAccelerationDegPerSecSq;
	Out.MaxYawJerkDegPerSecCubed = Constraints.MaxYawJerkDegPerSecCubed;
}

FAutopilotIntentHandle UAutopilotComponent::SubmitMovementIntent(const FAutopilotMovementIntent& Intent)
{
	if (ActiveRootMotionHandle.IsValid())
	{
		const FVector CurrentLocation = GetOwner()
			? GetOwner()->GetActorLocation() : FVector::ZeroVector;
		FAircraftAutopilotVehicleSnapshot RootMotionSnapshot;
		CaptureActiveRootMotionSnapshot(RootMotionSnapshot, 0.0f, CurrentLocation);
		MovementExecutor.FinishExternalIntent(
			ActiveRootMotionHandle,
			RootMotionSnapshot,
			EAutopilotIntentStatus::Interrupted,
			EAutopilotIntentFailureReason::Replaced);
		CleanupRootMotionIntent(true);
	}
	FAircraftAutopilotVehicleSnapshot Snapshot;
	const bool bHasControllerState = CaptureSnapshot(Snapshot);
	EAutopilotIntentFailureReason RejectionReason = EAutopilotIntentFailureReason::None;
	if (!FlightController.GetInterface() || !bHasControllerState)
	{
		RejectionReason = EAutopilotIntentFailureReason::FlightControllerUnavailable;
	}
	else if (!bAutopilotActive)
	{
		RejectionReason = EAutopilotIntentFailureReason::AutopilotInactive;
	}
	else if (Intent.Type == EAutopilotMovementIntentType::RootMotion)
	{
		RejectionReason = EAutopilotIntentFailureReason::InvalidIntent;
	}
	const FAutopilotIntentHandle Handle = MovementExecutor.Submit(Intent, Snapshot, RejectionReason);
	ApplyIntentMotionLimits();
	BroadcastIntentEvents();
	return Handle;
}

FAutopilotIntentHandle UAutopilotComponent::SubmitMoveTo(const FAutopilotMoveToCommand& Command)
{
	FAutopilotMovementIntent Intent;
	Intent.Type = EAutopilotMovementIntentType::MoveToPosition;
	Intent.TargetPositionCm = Command.TargetPositionCm;
	Intent.TargetActor = Command.TargetActor;
	ApplyFiniteOptions(Intent, Command.Options);
	return SubmitMovementIntent(Intent);
}

FAutopilotIntentHandle UAutopilotComponent::SubmitFollowPath(const FAutopilotFollowPathCommand& Command)
{
	FAutopilotMovementIntent Intent;
	Intent.Type = EAutopilotMovementIntentType::FollowPath;
	Intent.PathPointsCm = Command.PathPointsCm;
	Intent.PathTrajectoryMode = Command.TrajectoryMode;
	ApplyFiniteOptions(Intent, Command.Options);
	return SubmitMovementIntent(Intent);
}

FAutopilotIntentHandle UAutopilotComponent::SubmitOrbit(const FAutopilotOrbitCommand& Command)
{
	FAutopilotMovementIntent Intent;
	Intent.Type = EAutopilotMovementIntentType::Orbit;
	Intent.TargetPositionCm = Command.CenterPositionCm;
	Intent.TargetActor = Command.CenterActor;
	Intent.OrbitRadiusCm = Command.RadiusCm;
	Intent.OrbitAngularRateDegPerSec = Command.AngularRateDegPerSec;
	const float CommandedSpeed = FMath::Abs(Command.AngularRateDegPerSec) * (PI / 180.0f) * Command.RadiusCm;
	ApplyContinuousConstraints(Intent, Command.MotionConstraints, CommandedSpeed);
	ApplyHeadingOptions(Intent, Command.Heading);
	Intent.TimeoutSeconds = Command.TimeoutSeconds;
	return SubmitMovementIntent(Intent);
}

FAutopilotIntentHandle UAutopilotComponent::SubmitCircleArc(const FAutopilotCircleArcCommand& Command)
{
	FAutopilotMovementIntent Intent;
	Intent.Type = EAutopilotMovementIntentType::CircleArc;
	Intent.TargetPositionCm = Command.CenterPositionCm;
	Intent.TargetActor = Command.CenterActor;
	Intent.OrbitRadiusCm = Command.RadiusCm;
	Intent.ArcStartAngleDegrees = Command.StartAngleDegrees;
	Intent.ArcEndAngleDegrees = Command.EndAngleDegrees;
	ApplyFiniteOptions(Intent, Command.Options);
	return SubmitMovementIntent(Intent);
}

FAutopilotIntentHandle UAutopilotComponent::SubmitVelocity(const FAutopilotVelocityCommand& Command)
{
	FAutopilotMovementIntent Intent;
	Intent.Type = EAutopilotMovementIntentType::MoveWithVelocity;
	Intent.DesiredVelocityCmPerSec = Command.DesiredVelocityCmPerSec;
	ApplyContinuousConstraints(Intent, Command.MotionConstraints,
		FVector2D(Command.DesiredVelocityCmPerSec.X, Command.DesiredVelocityCmPerSec.Y).Size());
	ApplyHeadingOptions(Intent, Command.Heading);
	Intent.TimeoutSeconds = Command.TimeoutSeconds;
	return SubmitMovementIntent(Intent);
}

FAutopilotIntentHandle UAutopilotComponent::SubmitHold(const FAutopilotHeadingOptions& Heading)
{
	FAutopilotMovementIntent Intent;
	Intent.Type = EAutopilotMovementIntentType::Hold;
	ApplyHeadingOptions(Intent, Heading);
	return SubmitMovementIntent(Intent);
}

bool UAutopilotComponent::UpdateMovementIntent(FAutopilotIntentHandle Handle, const FAutopilotMovementIntent& Intent)
{
	const bool bUpdated = MovementExecutor.Update(Handle, Intent);
	if (bUpdated)
	{
		ApplyIntentMotionLimits();
	}
	return bUpdated;
}

bool UAutopilotComponent::UpdateMoveTo(FAutopilotIntentHandle Handle, const FAutopilotMoveToCommand& Command)
{
	FAutopilotMovementIntent Intent = MovementExecutor.GetActiveIntent();
	Intent.Type = EAutopilotMovementIntentType::MoveToPosition;
	Intent.TargetPositionCm = Command.TargetPositionCm;
	Intent.TargetActor = Command.TargetActor;
	ApplyFiniteOptions(Intent, Command.Options);
	return UpdateMovementIntent(Handle, Intent);
}

bool UAutopilotComponent::UpdateFollowPath(FAutopilotIntentHandle Handle, const FAutopilotFollowPathCommand& Command)
{
	FAutopilotMovementIntent Intent = MovementExecutor.GetActiveIntent();
	Intent.Type = EAutopilotMovementIntentType::FollowPath;
	Intent.PathPointsCm = Command.PathPointsCm;
	Intent.PathTrajectoryMode = Command.TrajectoryMode;
	ApplyFiniteOptions(Intent, Command.Options);
	return UpdateMovementIntent(Handle, Intent);
}

bool UAutopilotComponent::UpdateOrbit(FAutopilotIntentHandle Handle, const FAutopilotOrbitCommand& Command)
{
	FAutopilotMovementIntent Intent = MovementExecutor.GetActiveIntent();
	Intent.Type = EAutopilotMovementIntentType::Orbit;
	Intent.TargetPositionCm = Command.CenterPositionCm;
	Intent.TargetActor = Command.CenterActor;
	Intent.OrbitRadiusCm = Command.RadiusCm;
	Intent.OrbitAngularRateDegPerSec = Command.AngularRateDegPerSec;
	const float CommandedSpeed = FMath::Abs(Command.AngularRateDegPerSec) * (PI / 180.0f) * Command.RadiusCm;
	ApplyContinuousConstraints(Intent, Command.MotionConstraints, CommandedSpeed);
	ApplyHeadingOptions(Intent, Command.Heading);
	Intent.TimeoutSeconds = Command.TimeoutSeconds;
	return UpdateMovementIntent(Handle, Intent);
}

bool UAutopilotComponent::UpdateCircleArc(FAutopilotIntentHandle Handle, const FAutopilotCircleArcCommand& Command)
{
	FAutopilotMovementIntent Intent = MovementExecutor.GetActiveIntent();
	Intent.Type = EAutopilotMovementIntentType::CircleArc;
	Intent.TargetPositionCm = Command.CenterPositionCm;
	Intent.TargetActor = Command.CenterActor;
	Intent.OrbitRadiusCm = Command.RadiusCm;
	Intent.ArcStartAngleDegrees = Command.StartAngleDegrees;
	Intent.ArcEndAngleDegrees = Command.EndAngleDegrees;
	ApplyFiniteOptions(Intent, Command.Options);
	return UpdateMovementIntent(Handle, Intent);
}

bool UAutopilotComponent::UpdateVelocity(FAutopilotIntentHandle Handle, const FAutopilotVelocityCommand& Command)
{
	FAutopilotMovementIntent Intent = MovementExecutor.GetActiveIntent();
	Intent.Type = EAutopilotMovementIntentType::MoveWithVelocity;
	Intent.DesiredVelocityCmPerSec = Command.DesiredVelocityCmPerSec;
	ApplyContinuousConstraints(Intent, Command.MotionConstraints,
		FVector2D(Command.DesiredVelocityCmPerSec.X, Command.DesiredVelocityCmPerSec.Y).Size());
	ApplyHeadingOptions(Intent, Command.Heading);
	Intent.TimeoutSeconds = Command.TimeoutSeconds;
	return UpdateMovementIntent(Handle, Intent);
}

bool UAutopilotComponent::UpdateHeadingTarget(FAutopilotIntentHandle Handle, const FAutopilotHeadingOptions& Heading)
{
	FAutopilotMovementIntent Intent = MovementExecutor.GetActiveIntent();
	ApplyHeadingOptions(Intent, Heading);
	// 仅航向变化不重建轨迹
	return MovementExecutor.Update(Handle, Intent);
}

bool UAutopilotComponent::CancelMovementIntent(FAutopilotIntentHandle Handle)
{
	FAircraftAutopilotVehicleSnapshot Snapshot;
	if (!CaptureSnapshot(Snapshot))
	{
		return false;
	}
	return MovementExecutor.Cancel(Handle, Snapshot);
}

FAutopilotIntentResult UAutopilotComponent::GetIntentResult(FAutopilotIntentHandle Handle) const
{
	return MovementExecutor.GetResult(Handle);
}

FAutopilotIntentResult UAutopilotComponent::GetCurrentIntentResult() const
{
	return MovementExecutor.GetCurrentResult();
}

float UAutopilotComponent::GetTrajectoryProgress() const
{
	return MovementExecutor.GetTrajectoryProgress();
}

float UAutopilotComponent::GetEstimatedHoverThrust() const
{
	return HoverThrustEstimator.GetHoverThrust();
}

/* ---------------------------------------------------------------------------
 * Root Motion 桥接（Montage 根位移 → 运动目标 + 驱动覆盖）
 * ------------------------------------------------------------------------- */

bool UAutopilotComponent::PlayMontage(const FAutopilotMontagePlayback& Playback, FAutopilotIntentHandle& OutRootMotionHandle)
{
	OutRootMotionHandle = FAutopilotIntentHandle();
	AActor* Owner = GetOwner();
	USkeletalMeshComponent* SkeletalMesh = Owner
		? Cast<USkeletalMeshComponent>(Owner->GetRootComponent()) : nullptr;
	UAnimInstance* AnimInstance = SkeletalMesh ? SkeletalMesh->GetAnimInstance() : nullptr;
	if (!SkeletalMesh
		|| !AnimInstance
		|| !Playback.Montage
		|| !FMath::IsFinite(Playback.PlayRate)
		|| Playback.PlayRate <= UE_SMALL_NUMBER
		|| !FMath::IsFinite(Playback.StartPositionSeconds)
		|| Playback.StartPositionSeconds < 0.0f
		|| Playback.StartPositionSeconds >= Playback.Montage->GetPlayLength())
	{
		return false;
	}

	if (!Playback.Montage->HasRootMotion())
	{
		if (ActiveRootMotionHandle.IsValid())
		{
			return false;
		}
		return AnimInstance->Montage_Play(
			Playback.Montage,
			Playback.PlayRate,
			EMontagePlayReturnType::MontageLength,
			Playback.StartPositionSeconds,
			Playback.bStopAllMontages) > 0.0f;
	}

	FAutopilotMovementIntent Intent;
	Intent.Type = EAutopilotMovementIntentType::RootMotion;
	if (SimulationBudget.DriveMode == EAircraftSimulationDriveMode::None)
	{
		return false;
	}
	if (SimulationBudget.DriveMode == EAircraftSimulationDriveMode::FlightController)
	{
		Intent.MotionConstraints = FTrajectoryMotionConstraints();
		Intent.HeadingMode = EAutopilotHeadingMode::KeepCurrent;
	}
	OutRootMotionHandle = SubmitRootMotionRequest(
		Playback, Intent, SimulationBudget.DriveMode, true);
	return MovementExecutor.GetResult(OutRootMotionHandle).Status
		== EAutopilotIntentStatus::Accepted;
}

FAutopilotIntentHandle UAutopilotComponent::SubmitRootMotionKinematic(const FAutopilotKinematicRootMotionCommand& Command)
{
	FAutopilotMovementIntent Intent;
	Intent.Type = EAutopilotMovementIntentType::RootMotion;
	Intent.ArrivalCriteria = Command.ArrivalCriteria;
	Intent.TimeoutSeconds = Command.TimeoutSeconds;
	return SubmitRootMotionRequest(
		Command.Playback, Intent, EAircraftSimulationDriveMode::Kinematic,
		Command.bApplyRootMotionRotation);
}

FAutopilotIntentHandle UAutopilotComponent::SubmitRootMotionFlightController(const FAutopilotFlightControllerRootMotionCommand& Command)
{
	FAutopilotMovementIntent Intent;
	Intent.Type = EAutopilotMovementIntentType::RootMotion;
	Intent.MotionConstraints = Command.MotionConstraints;
	Intent.ArrivalCriteria = Command.ArrivalCriteria;
	Intent.TimeoutSeconds = Command.TimeoutSeconds;
	if (Command.bApplyRootMotionRotation)
	{
		Intent.HeadingMode = EAutopilotHeadingMode::KeepCurrent;
	}
	else
	{
		ApplyHeadingOptions(Intent, Command.Heading);
	}
	return SubmitRootMotionRequest(
		Command.Playback, Intent, EAircraftSimulationDriveMode::FlightController,
		Command.bApplyRootMotionRotation);
}

FAutopilotIntentHandle UAutopilotComponent::SubmitRootMotionPhysicsConstraint(const FAutopilotPhysicsConstraintRootMotionCommand& Command)
{
	FAutopilotMovementIntent Intent;
	Intent.Type = EAutopilotMovementIntentType::RootMotion;
	Intent.ArrivalCriteria = Command.ArrivalCriteria;
	Intent.TimeoutSeconds = Command.TimeoutSeconds;
	if (Command.bApplyRootMotionRotation)
	{
		Intent.HeadingMode = EAutopilotHeadingMode::KeepCurrent;
	}
	else
	{
		ApplyHeadingOptions(Intent, Command.Heading);
	}
	return SubmitRootMotionRequest(
		Command.Playback, Intent, EAircraftSimulationDriveMode::PhysicsConstraint,
		Command.bApplyRootMotionRotation);
}

FAutopilotIntentHandle UAutopilotComponent::SubmitRootMotionRequest(
	const FAutopilotMontagePlayback& Playback,
	const FAutopilotMovementIntent& Intent,
	EAircraftSimulationDriveMode DriveMode,
	bool bApplyRootMotionRotation)
{
	if (ActiveRootMotionHandle.IsValid())
	{
		const FVector CurrentLocation = GetOwner()
			? GetOwner()->GetActorLocation() : FVector::ZeroVector;
		FAircraftAutopilotVehicleSnapshot RootMotionSnapshot;
		CaptureActiveRootMotionSnapshot(RootMotionSnapshot, 0.0f, CurrentLocation);
		MovementExecutor.FinishExternalIntent(
			ActiveRootMotionHandle,
			RootMotionSnapshot,
			EAutopilotIntentStatus::Interrupted,
			EAutopilotIntentFailureReason::Replaced);
		CleanupRootMotionIntent(true);
	}

	USkeletalMeshComponent* SkeletalMesh = GetOwner()
		? Cast<USkeletalMeshComponent>(GetOwner()->GetRootComponent()) : nullptr;
	UAnimInstance* AnimInstance = SkeletalMesh ? SkeletalMesh->GetAnimInstance() : nullptr;
	FAircraftAutopilotVehicleSnapshot Snapshot;
	bool bHasControllerState = false;
	if (DriveMode == EAircraftSimulationDriveMode::Kinematic && GetOwner())
	{
		Snapshot = MakeRootMotionSnapshot(0.0f, GetOwner()->GetActorLocation());
		bHasControllerState = true;
	}
	else
	{
		bHasControllerState = CaptureSnapshot(Snapshot);
	}
	EAutopilotIntentFailureReason RejectionReason = EAutopilotIntentFailureReason::None;
	if (!FlightController.GetInterface() || !bHasControllerState)
	{
		RejectionReason = EAutopilotIntentFailureReason::FlightControllerUnavailable;
	}
	else if (!bAutopilotActive)
	{
		RejectionReason = EAutopilotIntentFailureReason::AutopilotInactive;
	}
	else if (!SkeletalMesh
		|| !GetOwner()
		|| !Playback.Montage
		|| !AnimInstance
		|| !Playback.Montage->HasRootMotion()
		|| !FMath::IsFinite(Intent.TimeoutSeconds)
		|| !FMath::IsFinite(Playback.PlayRate)
		|| Playback.PlayRate <= UE_SMALL_NUMBER
		|| !FMath::IsFinite(Playback.StartPositionSeconds)
		|| Playback.StartPositionSeconds < 0.0f
		|| Playback.StartPositionSeconds >= Playback.Montage->GetPlayLength())
	{
		RejectionReason = EAutopilotIntentFailureReason::InvalidIntent;
	}
	else if (!IsRootMotionArrivalFinite(Intent.ArrivalCriteria))
	{
		RejectionReason = EAutopilotIntentFailureReason::InvalidIntent;
	}
	else if (DriveMode == EAircraftSimulationDriveMode::FlightController
		&& !AreRootMotionConstraintsFinite(Intent.MotionConstraints))
	{
		RejectionReason = EAutopilotIntentFailureReason::InvalidIntent;
	}
	const FAutopilotIntentHandle Handle =
		MovementExecutor.Submit(Intent, Snapshot, RejectionReason);
	if (MovementExecutor.GetResult(Handle).Status != EAutopilotIntentStatus::Accepted)
	{
		BroadcastIntentEvents();
		return Handle;
	}

	ActiveRootMotionMesh = SkeletalMesh;
	ActiveRootMotionAnimInstance = AnimInstance;
	ActiveRootMotionMontage = Playback.Montage;
	ActiveRootMotionHandle = Handle;
	ActiveRootMotionDriveMode = DriveMode;
	bActiveRootMotionApplyRotation = bApplyRootMotionRotation;
	bRootMotionMontageEnded = false;
	bRootMotionMontageInterrupted = false;
	ActiveRootMotionStartPositionSeconds = Playback.StartPositionSeconds;
	ActiveRootMotionTargetPositionCm = Snapshot.PositionCm;
	PreviousRootMotionTargetPositionCm = Snapshot.PositionCm;
	ActiveRootMotionTrajectoryActorRotation = SkeletalMesh->GetComponentQuat();
	ActiveRootMotionDesiredActorRotation = ActiveRootMotionTrajectoryActorRotation;
	PreviousRootMotionDesiredActorRotation = ActiveRootMotionTrajectoryActorRotation;
	PreviousRootMotionTargetVelocityCmPerSec = Snapshot.VelocityCmPerSec;
	ActiveRootMotionTargetVelocityCmPerSec = FVector::ZeroVector;
	ActiveRootMotionTargetAccelerationCmPerSecSq = FVector::ZeroVector;
	ActiveRootMotionTargetAngularVelocityWorldDegPerSec = FVector::ZeroVector;
	PreviousRootMotionTargetYawDegrees = Snapshot.YawDegrees;
	RootMotionArrivalStableTimeSeconds = 0.0f;
	if (ActiveRootMotionDriveMode == EAircraftSimulationDriveMode::FlightController)
	{
		MotionProfile.Initialize(
			Snapshot.PositionCm,
			Snapshot.VelocityCmPerSec,
			Snapshot.AccelerationCmPerSecSq,
			Snapshot.YawDegrees,
			0.0f);
		ApplyIntentMotionLimits();
	}
	RefreshSimulationDriveSelection();
	if (SimulationBudget.DriveMode != ActiveRootMotionDriveMode)
	{
		MovementExecutor.FinishExternalIntent(
			Handle,
			Snapshot,
			EAutopilotIntentStatus::Failed,
			EAutopilotIntentFailureReason::InvalidIntent);
		CleanupRootMotionIntent(false);
		InvalidateOutputs();
		BroadcastIntentEvents();
		return Handle;
	}
	AddTickPrerequisiteComponent(ActiveRootMotionMesh);
	PrimaryComponentTick.TickInterval = 0.0f;
	SetComponentTickEnabled(true);
	InvalidateOutputs();

	const float MontageDuration = ActiveRootMotionAnimInstance->Montage_Play(
		ActiveRootMotionMontage,
		Playback.PlayRate,
		EMontagePlayReturnType::MontageLength,
		Playback.StartPositionSeconds,
		Playback.bStopAllMontages);
	if (MontageDuration <= 0.0f)
	{
		MovementExecutor.FinishExternalIntent(
			Handle,
			Snapshot,
			EAutopilotIntentStatus::Failed,
			EAutopilotIntentFailureReason::InvalidIntent);
		CleanupRootMotionIntent(false);
	}
	else
	{
		FOnMontageEnded EndDelegate;
		EndDelegate.BindUObject(this, &UAutopilotComponent::HandleRootMotionMontageEnded);
		ActiveRootMotionAnimInstance->Montage_SetEndDelegate(EndDelegate, ActiveRootMotionMontage);
		bRootMotionMontageEnded = false;
		bRootMotionMontageInterrupted = false;
	}
	BroadcastIntentEvents();
	return Handle;
}

void UAutopilotComponent::TickRootMotionIntent(float DeltaSeconds)
{
	if (!ActiveRootMotionMesh
		|| !ActiveRootMotionAnimInstance
		|| !ActiveRootMotionMontage
		|| !ActiveRootMotionHandle.IsValid()
		|| !GetOwner())
	{
		FAircraftAutopilotVehicleSnapshot Snapshot;
		if (GetOwner())
		{
			CaptureActiveRootMotionSnapshot(
				Snapshot, DeltaSeconds, GetOwner()->GetActorLocation());
		}
		MovementExecutor.FinishExternalIntent(
			ActiveRootMotionHandle,
			Snapshot,
			EAutopilotIntentStatus::Failed,
			EAutopilotIntentFailureReason::InvalidIntent);
		CleanupRootMotionIntent(true);
		InvalidateOutputs();
		BroadcastIntentEvents();
		return;
	}
	if (SimulationBudget.DriveMode != ActiveRootMotionDriveMode)
	{
		FAircraftAutopilotVehicleSnapshot Snapshot;
		CaptureActiveRootMotionSnapshot(
			Snapshot, DeltaSeconds, GetOwner()->GetActorLocation());
		MovementExecutor.FinishExternalIntent(
			ActiveRootMotionHandle,
			Snapshot,
			EAutopilotIntentStatus::Failed,
			EAutopilotIntentFailureReason::InvalidIntent);
		CleanupRootMotionIntent(true);
		InvalidateOutputs();
		BroadcastIntentEvents();
		return;
	}

	const FVector PreviousLocation = GetOwner()->GetActorLocation();
	FTransform WorldRootMotion = FTransform::Identity;
	const bool bConsumedRootMotion =
		ConsumeRootMotionDelta(ActiveRootMotionMesh, WorldRootMotion);
	if (bConsumedRootMotion)
	{
		AccumulateRootMotionTarget(WorldRootMotion);
	}

	FAircraftAutopilotVehicleSnapshot Snapshot;
	if (!CaptureActiveRootMotionSnapshot(Snapshot, DeltaSeconds, PreviousLocation))
	{
		MovementExecutor.FinishExternalIntent(
			ActiveRootMotionHandle,
			Snapshot,
			EAutopilotIntentStatus::Failed,
			EAutopilotIntentFailureReason::FlightControllerUnavailable);
		CleanupRootMotionIntent(true);
		InvalidateOutputs();
		BroadcastIntentEvents();
		return;
	}
	UpdateRootMotionTarget(Snapshot, bConsumedRootMotion, DeltaSeconds);

	const float MontageLength = ActiveRootMotionMontage->GetPlayLength();
	const float MontagePosition = ActiveRootMotionAnimInstance->Montage_GetPosition(
		ActiveRootMotionMontage);
	const float RemainingMontageLength = MontageLength - ActiveRootMotionStartPositionSeconds;
	const float Progress = bRootMotionMontageEnded
		? 1.0f
		: (RemainingMontageLength > UE_SMALL_NUMBER
			? FMath::Clamp(
				(MontagePosition - ActiveRootMotionStartPositionSeconds) / RemainingMontageLength,
				0.0f,
				1.0f)
			: 0.0f);
	if (!MovementExecutor.TickExternalIntent(
		ActiveRootMotionHandle, Snapshot, DeltaSeconds, Progress))
	{
		MotionProfile.Initialize(
			Snapshot.PositionCm,
			Snapshot.VelocityCmPerSec,
			Snapshot.AccelerationCmPerSecSq,
			Snapshot.YawDegrees,
			0.0f);
		CleanupRootMotionIntent(true);
		InvalidateOutputs();
		BroadcastIntentEvents();
		return;
	}

	if (!bRootMotionMontageEnded
		&& ActiveRootMotionAnimInstance->Montage_GetIsStopped(ActiveRootMotionMontage))
	{
		bRootMotionMontageEnded = true;
		bRootMotionMontageInterrupted = true;
	}
	if (!bRootMotionMontageEnded)
	{
		return;
	}

	const FAutopilotIntentHandle CompletedHandle = ActiveRootMotionHandle;
	const bool bInterrupted = bRootMotionMontageInterrupted;
	if (!bInterrupted && !HasReachedRootMotionTarget(Snapshot, DeltaSeconds))
	{
		return;
	}

	const bool bFlightControllerDriven =
		ActiveRootMotionDriveMode == EAircraftSimulationDriveMode::FlightController;
	const FVector FinalTargetPositionCm = ActiveRootMotionTargetPositionCm;
	MovementExecutor.FinishExternalIntent(
		CompletedHandle,
		Snapshot,
		bInterrupted ? EAutopilotIntentStatus::Interrupted : EAutopilotIntentStatus::Succeeded,
		bInterrupted
			? EAutopilotIntentFailureReason::AnimationInterrupted
			: EAutopilotIntentFailureReason::None);
	if (!bInterrupted)
	{
		MovementExecutor.EnterHold(Snapshot, &FinalTargetPositionCm);
	}
	if (!bFlightControllerDriven || bInterrupted)
	{
		MotionProfile.Initialize(
			Snapshot.PositionCm,
			Snapshot.VelocityCmPerSec,
			Snapshot.AccelerationCmPerSecSq,
			Snapshot.YawDegrees,
			0.0f);
		if (!bInterrupted)
		{
			CachedProfiledSetpoint = MotionProfile.GetCurrentSetpoint();
			CachedGuidanceCommand = FGuidanceCommand();
			CachedTurnCommand = FTurnCommand();
			CachedFeedForward = FFeedForward();
			if (CachedProfiledSetpoint.bValid)
			{
				FeedForwardCalculator.Compute(CachedProfiledSetpoint, CachedFeedForward);
			}
		}
	}
	CleanupRootMotionIntent(false);
	if (bInterrupted)
	{
		InvalidateOutputs();
	}
	BroadcastIntentEvents();
}

bool UAutopilotComponent::ConsumeRootMotionDelta(
	USkeletalMeshComponent* SkeletalMesh,
	FTransform& OutWorldRootMotion) const
{
	OutWorldRootMotion = FTransform::Identity;
	const AActor* Owner = GetOwner();
	if (!SkeletalMesh || !Owner || SkeletalMesh->GetOwner() != Owner)
	{
		return false;
	}
	const FRootMotionMovementParams RootMotion = SkeletalMesh->ConsumeRootMotion();
	if (!RootMotion.bHasRootMotion)
	{
		return false;
	}
	OutWorldRootMotion = SkeletalMesh->ConvertLocalRootMotionToWorld(
		RootMotion.GetRootMotionTransform());
	return !OutWorldRootMotion.ContainsNaN();
}

void UAutopilotComponent::AccumulateRootMotionTarget(const FTransform& WorldRootMotion)
{
	const AActor* Owner = GetOwner();
	if (!Owner)
	{
		return;
	}

	const FQuat ActualActorRotation = Owner->GetActorQuat();
	const FVector ActorLocalTranslation = ActualActorRotation.UnrotateVector(
		WorldRootMotion.GetTranslation());
	ActiveRootMotionTargetPositionCm +=
		ActiveRootMotionTrajectoryActorRotation.RotateVector(ActorLocalTranslation);

	const FQuat ActorLocalRotation = (
		ActualActorRotation.Inverse()
		* WorldRootMotion.GetRotation()
		* ActualActorRotation).GetNormalized();
	ActiveRootMotionTrajectoryActorRotation = (
		ActiveRootMotionTrajectoryActorRotation * ActorLocalRotation).GetNormalized();
}

void UAutopilotComponent::UpdateRootMotionTarget(
	const FAircraftAutopilotVehicleSnapshot& Snapshot,
	bool bConsumedRootMotion,
	float DeltaSeconds)
{
	if (DeltaSeconds <= UE_SMALL_NUMBER)
	{
		InvalidateOutputs();
		return;
	}

	FTrajectoryPoint NominalSetpoint;
	NominalSetpoint.PositionCm = ActiveRootMotionTargetPositionCm;
	ActiveRootMotionTargetVelocityCmPerSec = bConsumedRootMotion
		? (ActiveRootMotionTargetPositionCm - PreviousRootMotionTargetPositionCm) / DeltaSeconds
		: FVector::ZeroVector;
	ActiveRootMotionTargetAccelerationCmPerSecSq =
		(ActiveRootMotionTargetVelocityCmPerSec
			- PreviousRootMotionTargetVelocityCmPerSec) / DeltaSeconds;
	NominalSetpoint.VelocityCmPerSec = ActiveRootMotionTargetVelocityCmPerSec;
	NominalSetpoint.AccelerationCmPerSecSq = ActiveRootMotionTargetAccelerationCmPerSecSq;

	const FQuat ControlToBody = FlightController.GetInterface()
		? FlightController->GetAircraftControlToBodyRotation()
		: FQuat::Identity;
	const float RootMotionYawDegrees = (
		ActiveRootMotionTrajectoryActorRotation * ControlToBody).Rotator().Yaw;
	NominalSetpoint.YawDegrees = PreviousRootMotionTargetYawDegrees;
	if (bActiveRootMotionApplyRotation)
	{
		NominalSetpoint.YawDegrees = RootMotionYawDegrees;
		ActiveRootMotionDesiredActorRotation = ActiveRootMotionTrajectoryActorRotation;
		NominalSetpoint.YawRateDegreesPerSec = bConsumedRootMotion
			? FMath::FindDeltaAngleDegrees(
				PreviousRootMotionTargetYawDegrees, RootMotionYawDegrees) / DeltaSeconds
			: 0.0f;
	}
	else
	{
		MovementExecutor.ApplyHeading(Snapshot, NominalSetpoint);
		const FQuat DesiredControlWorld =
			FRotator(0.0f, NominalSetpoint.YawDegrees, 0.0f).Quaternion();
		ActiveRootMotionDesiredActorRotation = (
			DesiredControlWorld * ControlToBody.Inverse()).GetNormalized();
	}
	NominalSetpoint.bValid = true;

	FQuat DeltaRotation = (
		ActiveRootMotionDesiredActorRotation
		* PreviousRootMotionDesiredActorRotation.Inverse()).GetNormalized();
	if (DeltaRotation.W < 0.0f)
	{
		DeltaRotation.X *= -1.0f;
		DeltaRotation.Y *= -1.0f;
		DeltaRotation.Z *= -1.0f;
		DeltaRotation.W *= -1.0f;
	}
	FVector RotationAxis = FVector::UpVector;
	float RotationAngleRadians = 0.0f;
	DeltaRotation.ToAxisAndAngle(RotationAxis, RotationAngleRadians);
	ActiveRootMotionTargetAngularVelocityWorldDegPerSec =
		DeltaRotation.Equals(FQuat::Identity, UE_SMALL_NUMBER)
			? FVector::ZeroVector
			: FMath::RadiansToDegrees(
				RotationAxis.GetSafeNormal() * (RotationAngleRadians / DeltaSeconds));

	PreviousRootMotionTargetPositionCm = ActiveRootMotionTargetPositionCm;
	PreviousRootMotionTargetVelocityCmPerSec = ActiveRootMotionTargetVelocityCmPerSec;
	PreviousRootMotionTargetYawDegrees = NominalSetpoint.YawDegrees;
	PreviousRootMotionDesiredActorRotation = ActiveRootMotionDesiredActorRotation;

	if (ActiveRootMotionDriveMode != EAircraftSimulationDriveMode::FlightController)
	{
		InvalidateOutputs();
		return;
	}
	ApplyIntentMotionLimits();

	CachedGuidanceCommand = FGuidanceCommand();
	CachedTurnCommand = FTurnCommand();
	if (AutopilotConfig.bEnableCoordinatedTurns)
	{
		CachedTurnCommand = TurnBehavior.Compute(
			NominalSetpoint.VelocityCmPerSec,
			Snapshot.VelocityCmPerSec,
			Snapshot.YawDegrees,
			MovementExecutor.GetActiveIntent().MotionConstraints.MaxYawRateDegPerSec,
			DeltaSeconds);
	}

	CachedProfiledSetpoint = MotionProfile.Update(NominalSetpoint, DeltaSeconds);
	UpdateHoverThrustEstimate(DeltaSeconds);
	CachedFeedForward = FFeedForward();
	if (CachedProfiledSetpoint.bValid)
	{
		FeedForwardCalculator.Compute(CachedProfiledSetpoint, CachedFeedForward);
	}
}

bool UAutopilotComponent::HasReachedRootMotionTarget(
	const FAircraftAutopilotVehicleSnapshot& Snapshot,
	float DeltaSeconds)
{
	const FAutopilotArrivalCriteria& Criteria =
		MovementExecutor.GetActiveIntent().ArrivalCriteria;
	const FVector PositionError = ActiveRootMotionTargetPositionCm - Snapshot.PositionCm;
	const bool bUsesMotionProfile =
		ActiveRootMotionDriveMode == EAircraftSimulationDriveMode::FlightController;
	const FVector SetpointError = bUsesMotionProfile && CachedProfiledSetpoint.bValid
		? ActiveRootMotionTargetPositionCm - CachedProfiledSetpoint.PositionCm
		: PositionError;
	const bool bPositionReached =
		FVector2D(PositionError.X, PositionError.Y).Size() <= Criteria.HorizontalToleranceCm
		&& FMath::Abs(PositionError.Z) <= Criteria.VerticalToleranceCm;
	const bool bSetpointReached =
		FVector2D(SetpointError.X, SetpointError.Y).Size() <= Criteria.HorizontalToleranceCm
		&& FMath::Abs(SetpointError.Z) <= Criteria.VerticalToleranceCm;
	const bool bSpeedReached =
		Snapshot.VelocityCmPerSec.Size() <= Criteria.SpeedToleranceCmPerSec;
	const FQuat ControlToBody = FlightController.GetInterface()
		? FlightController->GetAircraftControlToBodyRotation()
		: FQuat::Identity;
	const float ConstraintTargetYawDegrees = (
		ActiveRootMotionDesiredActorRotation * ControlToBody).Rotator().Yaw;
	const float DesiredYawDegrees = bUsesMotionProfile && CachedProfiledSetpoint.bValid
		? CachedProfiledSetpoint.YawDegrees
		: ConstraintTargetYawDegrees;
	const bool bYawReached = FMath::Abs(FMath::FindDeltaAngleDegrees(
		Snapshot.YawDegrees, DesiredYawDegrees)) <= Criteria.YawToleranceDegrees;
	RootMotionArrivalStableTimeSeconds =
		bPositionReached && bSetpointReached && bSpeedReached && bYawReached
			? RootMotionArrivalStableTimeSeconds + FMath::Max(DeltaSeconds, 0.0f)
			: 0.0f;
	return RootMotionArrivalStableTimeSeconds >= Criteria.StableTimeSeconds;
}

void UAutopilotComponent::CleanupRootMotionIntent(bool bStopMontage)
{
	if (ActiveRootMotionAnimInstance)
	{
		if (ActiveRootMotionMontage)
		{
			if (FOnMontageEnded* EndDelegate =
				ActiveRootMotionAnimInstance->Montage_GetEndedDelegate(ActiveRootMotionMontage))
			{
				EndDelegate->Unbind();
			}
		}
		if (bStopMontage
			&& ActiveRootMotionMontage
			&& ActiveRootMotionAnimInstance->Montage_IsPlaying(ActiveRootMotionMontage))
		{
			ActiveRootMotionAnimInstance->Montage_Stop(0.0f, ActiveRootMotionMontage);
		}
	}
	if (ActiveRootMotionMesh)
	{
		RemoveTickPrerequisiteComponent(ActiveRootMotionMesh);
	}
	ActiveRootMotionMesh = nullptr;
	ActiveRootMotionAnimInstance = nullptr;
	ActiveRootMotionMontage = nullptr;
	ActiveRootMotionHandle = FAutopilotIntentHandle();
	ActiveRootMotionDriveMode = EAircraftSimulationDriveMode::FlightController;
	bActiveRootMotionApplyRotation = true;
	bRootMotionMontageEnded = false;
	bRootMotionMontageInterrupted = false;
	ActiveRootMotionTargetPositionCm = FVector::ZeroVector;
	PreviousRootMotionTargetPositionCm = FVector::ZeroVector;
	ActiveRootMotionTrajectoryActorRotation = FQuat::Identity;
	ActiveRootMotionDesiredActorRotation = FQuat::Identity;
	PreviousRootMotionDesiredActorRotation = FQuat::Identity;
	PreviousRootMotionTargetVelocityCmPerSec = FVector::ZeroVector;
	ActiveRootMotionTargetVelocityCmPerSec = FVector::ZeroVector;
	ActiveRootMotionTargetAccelerationCmPerSecSq = FVector::ZeroVector;
	ActiveRootMotionTargetAngularVelocityWorldDegPerSec = FVector::ZeroVector;
	PreviousRootMotionTargetYawDegrees = 0.0f;
	RootMotionArrivalStableTimeSeconds = 0.0f;
	ActiveRootMotionStartPositionSeconds = 0.0f;
	RefreshSimulationDriveSelection();
	RefreshSimulationTickEnabled();
}

bool UAutopilotComponent::CaptureActiveRootMotionSnapshot(
	FAircraftAutopilotVehicleSnapshot& OutSnapshot,
	float DeltaSeconds,
	const FVector& PreviousLocation) const
{
	if (ActiveRootMotionDriveMode != EAircraftSimulationDriveMode::Kinematic)
	{
		return CaptureSnapshot(OutSnapshot);
	}
	if (!GetOwner())
	{
		return false;
	}
	OutSnapshot = MakeRootMotionSnapshot(DeltaSeconds, PreviousLocation);
	return true;
}

FAircraftAutopilotVehicleSnapshot UAutopilotComponent::MakeRootMotionSnapshot(
	float DeltaSeconds, const FVector& PreviousLocation) const
{
	FAircraftAutopilotVehicleSnapshot Snapshot;
	const AActor* Owner = GetOwner();
	if (!Owner)
	{
		return Snapshot;
	}
	Snapshot.PositionCm = Owner->GetActorLocation();
	Snapshot.VelocityCmPerSec = DeltaSeconds > UE_SMALL_NUMBER
		? (Snapshot.PositionCm - PreviousLocation) / DeltaSeconds
		: FVector::ZeroVector;
	const FQuat ControlWorld = FlightController.GetInterface()
		? Owner->GetActorQuat() * FlightController->GetAircraftControlToBodyRotation()
		: Owner->GetActorQuat();
	Snapshot.YawDegrees = ControlWorld.Rotator().Yaw;
	return Snapshot;
}

void UAutopilotComponent::RefreshSimulationTickEnabled()
{
	const bool bRootMotionRequiresTick = ActiveRootMotionHandle.IsValid()
		&& !SimulationBudget.bIsNetworkProxy;
	const bool bBudgetAllowsTick = bRootMotionRequiresTick
		|| (SimulationBudget.bRunSlowLogic && !SimulationBudget.bIsNetworkProxy);
	PrimaryComponentTick.TickInterval = bRootMotionRequiresTick
		? 0.0f : FMath::Max(SimulationBudget.SlowLogicIntervalSeconds, 0.0f);
	SetComponentTickEnabled(bAutopilotActive && bBudgetAllowsTick);
}

void UAutopilotComponent::RefreshSimulationDriveSelection() const
{
	if (!GetOwner()) return;
	TArray<UActorComponent*> Components;
	GetOwner()->GetComponents(Components);
	for (UActorComponent* Component : Components)
	{
		if (Component
			&& Component->GetClass()->ImplementsInterface(
				UAircraftSimulationLODController::StaticClass()))
		{
			IAircraftSimulationLODController::Execute_RefreshAircraftSimulationDrive(Component);
		}
	}
}

void UAutopilotComponent::HandleRootMotionMontageEnded(UAnimMontage* Montage, bool bInterrupted)
{
	if (Montage == ActiveRootMotionMontage)
	{
		bRootMotionMontageEnded = true;
		bRootMotionMontageInterrupted = bInterrupted;
	}
}

/* ---------------------------------------------------------------------------
 * IAircraftSimulationLODConsumer
 * ------------------------------------------------------------------------- */

void UAutopilotComponent::ApplyAircraftSimulationBudget_Implementation(const FAircraftSimulationBudget& Budget)
{
	SimulationBudget = Budget;
	PrimaryComponentTick.TickInterval = ActiveRootMotionHandle.IsValid()
		? 0.0f : FMath::Max(Budget.SlowLogicIntervalSeconds, 0.0f);
	ResolveAutopilotConfig(/*bForceRefresh=*/true);
	RefreshSimulationTickEnabled();
}

bool UAutopilotComponent::GetAircraftMotionTarget_Implementation(FAircraftMotionTarget& OutTarget) const
{
	OutTarget = FAircraftMotionTarget();
	if (!bAutopilotActive) return false;
	if (ActiveRootMotionHandle.IsValid())
	{
		OutTarget.PositionCm = ActiveRootMotionTargetPositionCm;
		OutTarget.VelocityCmPerSec = ActiveRootMotionTargetVelocityCmPerSec;
		OutTarget.AccelerationCmPerSecSq = ActiveRootMotionTargetAccelerationCmPerSecSq;
		OutTarget.RotationDegrees = ActiveRootMotionDesiredActorRotation.Rotator();
		OutTarget.AngularVelocityWorldDegPerSec =
			ActiveRootMotionTargetAngularVelocityWorldDegPerSec;
		OutTarget.Priority = 1000;
		OutTarget.bValid = true;
		return true;
	}
	if (!CachedProfiledSetpoint.bValid) return false;
	OutTarget.PositionCm = CachedProfiledSetpoint.PositionCm;
	OutTarget.VelocityCmPerSec = CachedProfiledSetpoint.VelocityCmPerSec;
	OutTarget.AccelerationCmPerSecSq = CachedProfiledSetpoint.AccelerationCmPerSecSq;
	const FQuat DesiredControlWorld =
		FRotator(0.0f, CachedProfiledSetpoint.YawDegrees, 0.0f).Quaternion();
	const FQuat ControlToBody = FlightController.GetInterface()
		? FlightController->GetAircraftControlToBodyRotation()
		: FQuat::Identity;
	OutTarget.RotationDegrees = (DesiredControlWorld * ControlToBody.Inverse()).Rotator();
	OutTarget.AngularVelocityWorldDegPerSec =
		FVector(0.0f, 0.0f, CachedProfiledSetpoint.YawRateDegreesPerSec);
	OutTarget.Priority = 0;
	OutTarget.bValid = true;
	return true;
}

FAircraftSimulationDriveOverride UAutopilotComponent::GetAircraftSimulationDriveOverride_Implementation() const
{
	FAircraftSimulationDriveOverride Override;
	if (!ActiveRootMotionHandle.IsValid()) return Override;
	Override.DriveMode = ActiveRootMotionDriveMode;
	Override.Priority = 1000;
	Override.bValid = true;
	return Override;
}

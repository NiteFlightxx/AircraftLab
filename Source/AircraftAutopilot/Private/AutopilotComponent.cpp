// Copyright Epic Games, Inc. All Rights Reserved.

#include "AutopilotComponent.h"

#include "AutopilotDebugDraw.h"
#include "AutopilotMovementExecutor.h"
#include "DroneTypes.h"
#include "FeedForward/FeedForwardCalculator.h"
#include "FlightControllerComponent.h"
#include "MotionProfile/MotionProfile.h"
#include "PathFollowing/DirectGuidance.h"
#include "PathFollowing/PurePursuitGuidance.h"
#include "PathFollowing/VectorFieldGuidance.h"
#include "Trajectory/TrajectoryGenerator.h"

DEFINE_LOG_CATEGORY_STATIC(LogAutopilot, Log, All);

UAutopilotComponent::UAutopilotComponent()
{
	PrimaryComponentTick.bCanEverTick = true;
	PrimaryComponentTick.TickGroup = TG_PrePhysics;
}

void UAutopilotComponent::OnRegister()
{
	Super::OnRegister();
	CreateRuntimeObjects();
}

void UAutopilotComponent::BeginPlay()
{
	Super::BeginPlay();
	CreateRuntimeObjects();
	ResolveFlightController();
	ApplyProfile();
	if (FlightController)
	{
		FlightController->AddTickPrerequisiteComponent(this);
		const FDroneKinematicState& State = FlightController->GetEstimatedState().State;
		MotionProfile->Initialize(
			State.PositionCm,
			State.VelocityCmPerSec,
			State.AccelerationWorldCmPerSecSq,
			State.AttitudeDegrees.Yaw,
			State.AngularVelocityBodyDegreesPerSec.Z);
	}
}

void UAutopilotComponent::TickComponent(
	float DeltaTime,
	ELevelTick TickType,
	FActorComponentTickFunction* ThisTickFunction)
{
	Super::TickComponent(DeltaTime, TickType, ThisTickFunction);
	if (!bAutopilotActive || DeltaTime <= UE_SMALL_NUMBER)
	{
		return;
	}

	FAutopilotVehicleSnapshot Snapshot;
	if (!CaptureSnapshot(Snapshot))
	{
		InvalidateOutputs();
		return;
	}

	ApplyIntentMotionLimits();
	FTrajectoryPoint NominalSetpoint;
	if (!MovementExecutor->BuildSetpoint(
		Snapshot, DeltaTime, CachedProfiledSetpoint, NominalSetpoint))
	{
		InvalidateOutputs();
		BroadcastIntentEvents();
		return;
	}

	UTrajectoryGenerator* Trajectory = MovementExecutor->GetTrajectoryGenerator();
	const UAutopilotProfileAsset* EffectiveProfile = Profile ? Profile : GetDefault<UAutopilotProfileAsset>();
	const EAutopilotMovementIntentType IntentType = MovementExecutor->GetActiveIntent().Type;
	const bool bPathIntent = IntentType == EAutopilotMovementIntentType::FollowPath
		|| IntentType == EAutopilotMovementIntentType::Orbit
		|| IntentType == EAutopilotMovementIntentType::CircleArc;
	FGuidanceCommand Guidance;
	if (EffectiveProfile->bEnablePathFollowing && bPathIntent && PathFollowing
		&& Trajectory && Trajectory->IsValid())
	{
		PathFollowing->Update(Snapshot.PositionCm, Snapshot.VelocityCmPerSec, DeltaTime, Guidance);
		if (Guidance.bValid)
		{
			NominalSetpoint.VelocityCmPerSec.X = Guidance.DesiredVelocityCmPerSec.X;
			NominalSetpoint.VelocityCmPerSec.Y = Guidance.DesiredVelocityCmPerSec.Y;
		}
	}
	CachedGuidanceCommand = Guidance;
	MovementExecutor->ApplyHeading(Snapshot, NominalSetpoint);

	CachedTurnCommand = FTurnCommand();
	if (EffectiveProfile->bEnableCoordinatedTurns && TurnBehavior)
	{
		CachedTurnCommand = TurnBehavior->Compute(
			NominalSetpoint.VelocityCmPerSec,
			Snapshot.VelocityCmPerSec,
			Snapshot.YawDegrees,
			MovementExecutor->GetActiveIntent().MotionConstraints.MaxYawRateDegPerSec,
			DeltaTime);
	}

	CachedProfiledSetpoint = MotionProfile->Update(NominalSetpoint, DeltaTime);
	UpdateHoverThrustEstimate(DeltaTime);
	CachedFeedForward = FFeedForward();
	if (CachedProfiledSetpoint.bValid)
	{
		FeedForwardCalculator->Compute(CachedProfiledSetpoint, CachedFeedForward);
	}

	MovementExecutor->UpdateCompletion(Snapshot, DeltaTime, CachedProfiledSetpoint);
	BroadcastIntentEvents();
	FAutopilotDebugDraw::DrawAll(
		GetWorld(), Trajectory, NominalSetpoint, Guidance, Snapshot.PositionCm);
}

bool UAutopilotComponent::GetAutopilotInjection(FAutopilotInjection& OutInjection) const
{
	if (!bAutopilotActive || !CachedProfiledSetpoint.bValid)
	{
		OutInjection = FAutopilotInjection();
		return false;
	}
	BuildInjection(OutInjection);
	return OutInjection.bValid;
}

void UAutopilotComponent::SetAutopilotActive(bool bActive)
{
	if (bActive == bAutopilotActive && bActivationInitialized == bActive)
	{
		return;
	}
	if (!FlightController) ResolveFlightController();
	if (!FlightController)
	{
		UE_LOG(LogAutopilot, Error, TEXT("Cannot change Autopilot state without FlightController."));
		return;
	}

	bAutopilotActive = bActive;
	if (bActive)
	{
		if (!bFlightModeBeforeActivationCaptured)
		{
			FlightModeBeforeActivation = static_cast<uint8>(FlightController->GetFlightMode());
			bFlightModeBeforeActivationCaptured = true;
		}
		FlightController->SetFlightMode(EDroneFlightMode::Mission);
		FlightController->SetUseAutopilotSetpoint(true);
		bActivationInitialized = true;
		FAutopilotVehicleSnapshot Snapshot;
		if (CaptureSnapshot(Snapshot))
		{
			MotionProfile->Initialize(
				Snapshot.PositionCm,
				Snapshot.VelocityCmPerSec,
				Snapshot.AccelerationCmPerSecSq,
				Snapshot.YawDegrees,
				0.0f);
			MovementExecutor->EnterHold(Snapshot);
		}
	}
	else
	{
		MovementExecutor->CancelActive(EAutopilotIntentFailureReason::CancelledByCaller);
		BroadcastIntentEvents();
		FlightController->SetUseAutopilotSetpoint(false);
		if (bFlightModeBeforeActivationCaptured
			&& FlightController->GetFlightMode() == EDroneFlightMode::Mission)
		{
			FlightController->SetFlightMode(static_cast<EDroneFlightMode>(FlightModeBeforeActivation));
		}
		bFlightModeBeforeActivationCaptured = false;
		bActivationInitialized = false;
		MovementExecutor->GetTrajectoryGenerator()->Clear();
		InvalidateOutputs();
	}
}

FAutopilotIntentHandle UAutopilotComponent::SubmitMovementIntent(const FAutopilotMovementIntent& Intent)
{
	FAutopilotVehicleSnapshot Snapshot;
	const bool bHasControllerState = CaptureSnapshot(Snapshot);
	EAutopilotIntentFailureReason RejectionReason = EAutopilotIntentFailureReason::None;
	if (!FlightController || !bHasControllerState)
	{
		RejectionReason = EAutopilotIntentFailureReason::FlightControllerUnavailable;
	}
	else if (!bAutopilotActive)
	{
		RejectionReason = EAutopilotIntentFailureReason::AutopilotInactive;
	}
	const FAutopilotIntentHandle Handle = MovementExecutor->Submit(Intent, Snapshot, RejectionReason);
	ApplyIntentMotionLimits();
	BroadcastIntentEvents();
	return Handle;
}

void UAutopilotComponent::ApplyHeadingOptions(
	FAutopilotMovementIntent& Intent, const FAutopilotHeadingOptions& Heading)
{
	Intent.HeadingMode = Heading.Mode;
	Intent.FixedYawDegrees = Heading.FixedYawDegrees;
	Intent.bUseIndependentHeadingTarget = Heading.bUseLookAtTarget;
	Intent.HeadingTargetPositionCm = Heading.LookAtPositionCm;
	Intent.HeadingTargetActor = Heading.LookAtActor;
}

void UAutopilotComponent::ApplyFiniteOptions(
	FAutopilotMovementIntent& Intent, const FAutopilotFiniteCommandOptions& Options)
{
	Intent.MotionConstraints = Options.MotionConstraints;
	ApplyHeadingOptions(Intent, Options.Heading);
	Intent.ArrivalMode = Options.ArrivalMode;
	Intent.ArrivalCriteria = Options.ArrivalCriteria;
	Intent.TimeoutSeconds = Options.TimeoutSeconds;
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
	Intent.MotionConstraints = Command.MotionConstraints;
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
	Intent.MotionConstraints = Command.MotionConstraints;
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

bool UAutopilotComponent::UpdateMoveTo(
	FAutopilotIntentHandle Handle, const FAutopilotMoveToCommand& Command)
{
	FAutopilotMovementIntent Intent;
	Intent.Type = EAutopilotMovementIntentType::MoveToPosition;
	Intent.TargetPositionCm = Command.TargetPositionCm;
	Intent.TargetActor = Command.TargetActor;
	ApplyFiniteOptions(Intent, Command.Options);
	return UpdateMovementIntent(Handle, Intent);
}

bool UAutopilotComponent::UpdateFollowPath(
	FAutopilotIntentHandle Handle, const FAutopilotFollowPathCommand& Command)
{
	FAutopilotMovementIntent Intent;
	Intent.Type = EAutopilotMovementIntentType::FollowPath;
	Intent.PathPointsCm = Command.PathPointsCm;
	Intent.PathTrajectoryMode = Command.TrajectoryMode;
	ApplyFiniteOptions(Intent, Command.Options);
	return UpdateMovementIntent(Handle, Intent);
}

bool UAutopilotComponent::UpdateOrbit(
	FAutopilotIntentHandle Handle, const FAutopilotOrbitCommand& Command)
{
	FAutopilotMovementIntent Intent;
	Intent.Type = EAutopilotMovementIntentType::Orbit;
	Intent.TargetPositionCm = Command.CenterPositionCm;
	Intent.TargetActor = Command.CenterActor;
	Intent.OrbitRadiusCm = Command.RadiusCm;
	Intent.OrbitAngularRateDegPerSec = Command.AngularRateDegPerSec;
	Intent.MotionConstraints = Command.MotionConstraints;
	ApplyHeadingOptions(Intent, Command.Heading);
	Intent.TimeoutSeconds = Command.TimeoutSeconds;
	return UpdateMovementIntent(Handle, Intent);
}

bool UAutopilotComponent::UpdateCircleArc(
	FAutopilotIntentHandle Handle, const FAutopilotCircleArcCommand& Command)
{
	FAutopilotMovementIntent Intent;
	Intent.Type = EAutopilotMovementIntentType::CircleArc;
	Intent.TargetPositionCm = Command.CenterPositionCm;
	Intent.TargetActor = Command.CenterActor;
	Intent.OrbitRadiusCm = Command.RadiusCm;
	Intent.ArcStartAngleDegrees = Command.StartAngleDegrees;
	Intent.ArcEndAngleDegrees = Command.EndAngleDegrees;
	ApplyFiniteOptions(Intent, Command.Options);
	return UpdateMovementIntent(Handle, Intent);
}

bool UAutopilotComponent::UpdateVelocity(
	FAutopilotIntentHandle Handle, const FAutopilotVelocityCommand& Command)
{
	FAutopilotMovementIntent Intent;
	Intent.Type = EAutopilotMovementIntentType::MoveWithVelocity;
	Intent.DesiredVelocityCmPerSec = Command.DesiredVelocityCmPerSec;
	Intent.MotionConstraints = Command.MotionConstraints;
	ApplyHeadingOptions(Intent, Command.Heading);
	Intent.TimeoutSeconds = Command.TimeoutSeconds;
	return UpdateMovementIntent(Handle, Intent);
}

bool UAutopilotComponent::UpdateHeadingTarget(
	FAutopilotIntentHandle Handle, const FAutopilotHeadingOptions& Heading)
{
	if (!MovementExecutor || MovementExecutor->GetCurrentResult().Handle != Handle)
	{
		return false;
	}
	FAutopilotMovementIntent UpdatedIntent = MovementExecutor->GetActiveIntent();
	ApplyHeadingOptions(UpdatedIntent, Heading);
	return UpdateMovementIntent(Handle, UpdatedIntent);
}

bool UAutopilotComponent::UpdateMovementIntent(
	FAutopilotIntentHandle Handle,
	const FAutopilotMovementIntent& Intent)
{
	const bool bUpdated = MovementExecutor->Update(Handle, Intent);
	if (bUpdated) ApplyIntentMotionLimits();
	return bUpdated;
}

bool UAutopilotComponent::CancelMovementIntent(FAutopilotIntentHandle Handle)
{
	FAutopilotVehicleSnapshot Snapshot;
	if (!CaptureSnapshot(Snapshot)) return false;
	const bool bCancelled = MovementExecutor->Cancel(Handle, Snapshot);
	if (bCancelled)
	{
		ApplyIntentMotionLimits();
		BroadcastIntentEvents();
	}
	return bCancelled;
}

FAutopilotIntentResult UAutopilotComponent::GetIntentResult(FAutopilotIntentHandle Handle) const
{
	return MovementExecutor ? MovementExecutor->GetResult(Handle) : FAutopilotIntentResult();
}

FAutopilotIntentResult UAutopilotComponent::GetCurrentIntentResult() const
{
	return MovementExecutor ? MovementExecutor->GetCurrentResult() : FAutopilotIntentResult();
}

float UAutopilotComponent::GetTrajectoryProgress() const
{
	return MovementExecutor ? MovementExecutor->GetTrajectoryProgress() : 0.0f;
}

float UAutopilotComponent::GetEstimatedHoverThrust() const
{
	return HoverThrustEstimator.GetHoverThrust();
}

void UAutopilotComponent::CreateRuntimeObjects()
{
	if (!MovementExecutor)
	{
		MovementExecutor = NewObject<UAutopilotMovementExecutor>(this);
		MovementExecutor->Initialize();
	}
	if (!MotionProfile) MotionProfile = NewObject<UMotionProfile>(this);
	if (!FeedForwardCalculator) FeedForwardCalculator = NewObject<UFeedForwardCalculator>(this);
	if (!TurnBehavior) TurnBehavior = NewObject<UTurnBehavior>(this);
	if (!PathFollowing) SetPathFollowingStrategy(EPathFollowingStrategy::PurePursuit);
}

void UAutopilotComponent::ResolveFlightController()
{
	if (AActor* Owner = GetOwner())
	{
		FlightController = Owner->FindComponentByClass<UFlightControllerComponent>();
		if (FlightController) FlightController->SetAutopilotProvider(this);
	}
}

void UAutopilotComponent::ApplyProfile()
{
	const UAutopilotProfileAsset* EffectiveProfile = Profile ? Profile : GetDefault<UAutopilotProfileAsset>();
	FeedForwardCalculator->SetParams(EffectiveProfile->FeedForward);
	TurnBehavior->SetLimits(EffectiveProfile->TurnLimits);
	const float InitialHoverThrust = FlightController
		? FlightController->GetHoverCollectiveCommand() : 0.5f;
	HoverThrustEstimator.Configure(EffectiveProfile->HoverThrustEstimator, InitialHoverThrust);
	SetPathFollowingStrategy(EffectiveProfile->GuidanceStrategy);
}

void UAutopilotComponent::ApplyIntentMotionLimits()
{
	if (!MovementExecutor || !MotionProfile) return;
	const FTrajectoryMotionConstraints& Requested = MovementExecutor->GetActiveIntent().MotionConstraints;
	float HardHorizontalSpeed = TNumericLimits<float>::Max();
	float HardHorizontalAcceleration = TNumericLimits<float>::Max();
	if (FlightController)
	{
		const FDroneFlightControllerConfig& ControllerConfig =
			FlightController->GetRuntimeConfig().Controller;
		const FDroneControlLimits& HardLimits = ControllerConfig.Limits;
		HardHorizontalSpeed = HardLimits.MaxHorizontalSpeedCmPerSec;
		const float TiltLimitedAcceleration = FlightController->GetGravityMagnitudeCmPerSecSq()
			* FMath::Tan(FMath::DegreesToRadians(HardLimits.MaxTiltAngleDegrees));
		HardHorizontalAcceleration = FMath::Min(
			HardLimits.MaxHorizontalAccelerationCmPerSecSq, TiltLimitedAcceleration);
		const FlightControlDynamics::FDampingAwareHorizontalLimits DampingAwareLimits =
			FlightControlDynamics::ComputeDampingAwareHorizontalLimits(
				FMath::Min(HardHorizontalSpeed, Requested.CruiseSpeedCmPerSec),
				HardHorizontalAcceleration,
				FlightController->GetLinearDampingPerSecond(),
				ControllerConfig.Position.DampingAccelerationReserveFraction);
		HardHorizontalSpeed = DampingAwareLimits.MaxSpeedCmPerSec;
		HardHorizontalAcceleration = DampingAwareLimits.MaxTrajectoryAccelerationCmPerSecSq;
	}
	MovementExecutor->SetPhysicalMotionLimits(HardHorizontalSpeed, HardHorizontalAcceleration);
	FMotionProfileLimits Limits;
	Limits.MaxHorizontalSpeedCmPerSec = FMath::Min(
		Requested.CruiseSpeedCmPerSec, HardHorizontalSpeed);
	Limits.MaxHorizontalAccelCmPerSecSq = FMath::Min3(
		Requested.MaxAccelerationCmPerSecSq,
		Requested.MaxDecelerationCmPerSecSq,
		HardHorizontalAcceleration);
	Limits.MaxHorizontalJerkCmPerSecCubed = Requested.MaxJerkCmPerSecCubed;
	Limits.MaxClimbRateCmPerSec = Requested.MaxClimbRateCmPerSec;
	Limits.MaxDescentRateCmPerSec = Requested.MaxDescentRateCmPerSec;
	Limits.MaxVerticalAccelCmPerSecSq = Requested.MaxVerticalAccelerationCmPerSecSq;
	Limits.MaxVerticalJerkCmPerSecCubed = Requested.MaxVerticalJerkCmPerSecCubed;
	Limits.MaxYawRateDegPerSec = Requested.MaxYawRateDegPerSec;
	Limits.MaxYawAccelDegPerSecSq = Requested.MaxYawAccelerationDegPerSecSq;
	Limits.MaxYawJerkDegPerSecCubed = Requested.MaxYawJerkDegPerSecCubed;
	Limits.MaxRollRateDegPerSec = Requested.MaxRollRateDegPerSec;
	Limits.MaxPitchRateDegPerSec = Requested.MaxPitchRateDegPerSec;
	MotionProfile->SetLimits(Limits);
}

void UAutopilotComponent::SetPathFollowingStrategy(EPathFollowingStrategy Strategy)
{
	switch (Strategy)
	{
	case EPathFollowingStrategy::PurePursuit:
		PathFollowing = NewObject<UPurePursuitGuidance>(this);
		break;
	case EPathFollowingStrategy::VectorField:
		PathFollowing = NewObject<UVectorFieldGuidance>(this);
		break;
	default:
		PathFollowing = NewObject<UDirectGuidance>(this);
		break;
	}
	if (!PathFollowing || !MovementExecutor) return;
	PathFollowing->SetTrajectory(MovementExecutor->GetTrajectoryGenerator());
	const UAutopilotProfileAsset* EffectiveProfile = Profile ? Profile : GetDefault<UAutopilotProfileAsset>();
	if (UPurePursuitGuidance* PurePursuit = Cast<UPurePursuitGuidance>(PathFollowing))
	{
		PurePursuit->SetConfig(EffectiveProfile->PurePursuit);
	}
	else if (UVectorFieldGuidance* VectorField = Cast<UVectorFieldGuidance>(PathFollowing))
	{
		VectorField->SetConfig(EffectiveProfile->VectorField);
	}
}

bool UAutopilotComponent::CaptureSnapshot(FAutopilotVehicleSnapshot& OutSnapshot) const
{
	if (!FlightController) return false;
	const FDroneKinematicState& State = FlightController->GetEstimatedState().State;
	OutSnapshot.PositionCm = State.PositionCm;
	OutSnapshot.VelocityCmPerSec = State.VelocityCmPerSec;
	OutSnapshot.AccelerationCmPerSecSq = State.AccelerationWorldCmPerSecSq;
	OutSnapshot.YawDegrees = State.AttitudeDegrees.Yaw;
	return true;
}

void UAutopilotComponent::BroadcastIntentEvents()
{
	TArray<FAutopilotIntentResult> Started;
	TArray<FAutopilotIntentResult> Finished;
	MovementExecutor->DrainEvents(Started, Finished);
	for (const FAutopilotIntentResult& Result : Started) OnIntentStarted.Broadcast(Result);
	for (const FAutopilotIntentResult& Result : Finished) OnIntentFinished.Broadcast(Result);
}

void UAutopilotComponent::InvalidateOutputs()
{
	CachedProfiledSetpoint = FProfiledSetpoint();
	CachedFeedForward = FFeedForward();
	CachedGuidanceCommand = FGuidanceCommand();
	CachedTurnCommand = FTurnCommand();
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
	OutInjection.YawRateSetpointDegPerSec = CachedProfiledSetpoint.YawRateDegreesPerSec
		+ CachedTurnCommand.DesiredYawRateDegPerSec;
	OutInjection.TurnRollDegrees = CachedTurnCommand.DesiredRollDegrees;
	OutInjection.bValid = true;
}

void UAutopilotComponent::UpdateHoverThrustEstimate(float DeltaSeconds)
{
	const UAutopilotProfileAsset* EffectiveProfile = Profile ? Profile : GetDefault<UAutopilotProfileAsset>();
	if (!FlightController)
	{
		return;
	}
	const float GravityCmPerSecSq = FlightController->GetGravityMagnitudeCmPerSecSq();
	float HoverThrust = FlightController->GetHoverCollectiveCommand();
	if (!EffectiveProfile->bEnableHoverThrustEstimator)
	{
		FeedForwardCalculator->SetPhysicalReference(GravityCmPerSecSq, HoverThrust);
		return;
	}
	const float AccelerationMpsSq = FlightController->GetEstimatedState()
		.State.AccelerationWorldCmPerSecSq.Z * 0.01f;
	const float CollectiveThrust = FlightController->GetControlOutput()
		.Targets.Attitude.CollectiveThrust;
	HoverThrustEstimator.Update(
		DeltaSeconds, AccelerationMpsSq, CollectiveThrust, GravityCmPerSecSq * 0.01f);
	HoverThrust = HoverThrustEstimator.GetHoverThrust();
	FeedForwardCalculator->SetPhysicalReference(GravityCmPerSecSq, HoverThrust);
}

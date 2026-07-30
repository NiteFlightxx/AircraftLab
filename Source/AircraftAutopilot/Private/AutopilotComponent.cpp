// Copyright Epic Games, Inc. All Rights Reserved.

#include "AutopilotComponent.h"

#include "AutopilotDebugDraw.h"
#include "AutopilotMovementExecutor.h"
#include "Animation/AnimInstance.h"
#include "Animation/AnimMontage.h"
#include "Components/SkeletalMeshComponent.h"
#include "FeedForward/FeedForwardCalculator.h"
#include "MotionProfile/MotionProfile.h"
#include "PathFollowing/DirectGuidance.h"
#include "PathFollowing/PurePursuitGuidance.h"
#include "PathFollowing/VectorFieldGuidance.h"
#include "Trajectory/TrajectoryGenerator.h"

DEFINE_LOG_CATEGORY_STATIC(LogAutopilot, Log, All);

struct FAutopilotRootMotionRequest
{
	const FAutopilotMontagePlayback* Playback = nullptr;
	FAutopilotMovementIntent Intent;
	EAircraftSimulationDriveMode DriveMode =
		EAircraftSimulationDriveMode::FlightController;
	bool bApplyRootMotionRotation = true;
};

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

bool UAutopilotComponent::PlayMontage(
	const FAutopilotMontagePlayback& Playback,
	FAutopilotIntentHandle& OutRootMotionHandle)
{
	OutRootMotionHandle = FAutopilotIntentHandle();
	CreateRuntimeObjects();
	AActor* Owner = GetOwner();
	USkeletalMeshComponent* SkeletalMesh = Owner
		? Cast<USkeletalMeshComponent>(Owner->GetRootComponent()) : nullptr;
	UAnimInstance* AnimInstance =
		SkeletalMesh ? SkeletalMesh->GetAnimInstance() : nullptr;
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

	FAutopilotRootMotionRequest Request;
	Request.Playback = &Playback;
	Request.Intent.Type = EAutopilotMovementIntentType::RootMotion;
	Request.DriveMode = SimulationBudget.DriveMode;
	Request.bApplyRootMotionRotation = true;
	if (Request.DriveMode == EAircraftSimulationDriveMode::None)
	{
		return false;
	}
	if (Request.DriveMode == EAircraftSimulationDriveMode::FlightController)
	{
		Request.Intent.MotionConstraints = FTrajectoryMotionConstraints();
		Request.Intent.HeadingMode = EAutopilotHeadingMode::KeepCurrent;
	}
	OutRootMotionHandle = SubmitRootMotionRequest(Request);
	return MovementExecutor
		&& MovementExecutor->GetResult(OutRootMotionHandle).Status
			== EAutopilotIntentStatus::Accepted;
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
	if (FlightController && FlightControllerComponent)
	{
		FlightControllerComponent->AddTickPrerequisiteComponent(this);
		FAircraftFlightKinematicState State;
		if (FlightController->GetAircraftFlightKinematicState(State))
		{
			MotionProfile->Initialize(
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
	CleanupRootMotionIntent(true);
	Super::EndPlay(EndPlayReason);
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
	if (ActiveRootMotionHandle.IsValid())
	{
		TickRootMotionIntent(DeltaTime);
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
	if (EffectiveProfile->GuidanceStrategy != EPathFollowingStrategy::Direct
		&& bPathIntent && PathFollowing
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
	const bool bRootMotionBypassesFlightController = ActiveRootMotionHandle.IsValid()
		&& ActiveRootMotionDriveMode
			!= EAircraftSimulationDriveMode::FlightController;
	if (!bAutopilotActive || bRootMotionBypassesFlightController
		|| !CachedProfiledSetpoint.bValid)
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
			FlightModeBeforeActivation = FlightController->ActivateAircraftAutopilotControl();
			bFlightModeBeforeActivationCaptured = true;
		}
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
		CleanupRootMotionIntent(true);
		MovementExecutor->CancelActive(EAutopilotIntentFailureReason::CancelledByCaller);
		BroadcastIntentEvents();
		if (bFlightModeBeforeActivationCaptured)
		{
			FlightController->DeactivateAircraftAutopilotControl(FlightModeBeforeActivation);
		}
		bFlightModeBeforeActivationCaptured = false;
		bActivationInitialized = false;
		MovementExecutor->GetTrajectoryGenerator()->Clear();
		InvalidateOutputs();
	}
	RefreshSimulationTickEnabled();
}

void UAutopilotComponent::SetProfileAsset(UAutopilotProfileAsset* InProfile)
{
	Profile = InProfile;
	if (HasBegunPlay())
	{
		ApplyProfile();
		ApplyIntentMotionLimits();
	}
}

void UAutopilotComponent::ApplyAircraftSimulationBudget_Implementation(
	const FAircraftSimulationBudget& Budget)
{
	SimulationBudget = Budget;
	PrimaryComponentTick.TickInterval = ActiveRootMotionHandle.IsValid()
		? 0.0f : FMath::Max(Budget.SlowLogicIntervalSeconds, 0.0f);
	RefreshSimulationTickEnabled();
}

bool UAutopilotComponent::GetAircraftMotionTarget_Implementation(
	FAircraftMotionTarget& OutTarget) const
{
	OutTarget = FAircraftMotionTarget();
	if (!bAutopilotActive) return false;
	if (ActiveRootMotionHandle.IsValid())
	{
		OutTarget.PositionCm = ActiveRootMotionTargetPositionCm;
		OutTarget.VelocityCmPerSec =
			ActiveRootMotionTargetVelocityCmPerSec;
		OutTarget.AccelerationCmPerSecSq =
			ActiveRootMotionTargetAccelerationCmPerSecSq;
		OutTarget.RotationDegrees =
			ActiveRootMotionDesiredActorRotation.Rotator();
		OutTarget.AngularVelocityWorldDegPerSec =
			ActiveRootMotionTargetAngularVelocityWorldDegPerSec;
		OutTarget.Priority = 1000;
		OutTarget.bValid = true;
		return true;
	}
	if (!CachedProfiledSetpoint.bValid) return false;
	OutTarget.PositionCm = CachedProfiledSetpoint.PositionCm;
	OutTarget.VelocityCmPerSec =
		CachedProfiledSetpoint.VelocityCmPerSec;
	OutTarget.AccelerationCmPerSecSq =
		CachedProfiledSetpoint.AccelerationCmPerSecSq;
	const FQuat DesiredControlWorld =
		FRotator(0.0f, CachedProfiledSetpoint.YawDegrees, 0.0f).Quaternion();
	const FQuat ControlToBody = FlightController
		? FlightController->GetAircraftControlToBodyRotation()
		: FQuat::Identity;
	OutTarget.RotationDegrees = (DesiredControlWorld * ControlToBody.Inverse()).Rotator();
	OutTarget.AngularVelocityWorldDegPerSec =
		FVector(0.0f, 0.0f, CachedProfiledSetpoint.YawRateDegreesPerSec);
	OutTarget.Priority = 0;
	OutTarget.bValid = true;
	return true;
}

FAircraftSimulationDriveOverride
UAutopilotComponent::GetAircraftSimulationDriveOverride_Implementation() const
{
	FAircraftSimulationDriveOverride Override;
	if (!ActiveRootMotionHandle.IsValid()) return Override;
	Override.DriveMode = ActiveRootMotionDriveMode;
	Override.Priority = 1000;
	Override.bValid = true;
	return Override;
}

FAutopilotIntentHandle UAutopilotComponent::SubmitMovementIntent(const FAutopilotMovementIntent& Intent)
{
	if (ActiveRootMotionHandle.IsValid())
	{
		const FVector CurrentLocation = GetOwner()
			? GetOwner()->GetActorLocation() : FVector::ZeroVector;
		FAutopilotVehicleSnapshot RootMotionSnapshot;
		CaptureActiveRootMotionSnapshot(
			RootMotionSnapshot, 0.0f, CurrentLocation);
		MovementExecutor->FinishExternalIntent(
			ActiveRootMotionHandle,
			RootMotionSnapshot,
			EAutopilotIntentStatus::Interrupted,
			EAutopilotIntentFailureReason::Replaced);
		CleanupRootMotionIntent(true);
	}
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
	else if (Intent.Type == EAutopilotMovementIntentType::RootMotion)
	{
		// Root Motion requires Mesh/Montage runtime context and a concrete drive path.
		RejectionReason = EAutopilotIntentFailureReason::InvalidIntent;
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

void UAutopilotComponent::ApplyFiniteOptions(
	FAutopilotMovementIntent& Intent, const FAutopilotFiniteCommandOptions& Options)
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
	// Continuous commands use one symmetric slew limit; they have no terminal braking phase.
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
	ApplyContinuousConstraints(Intent, Command.MotionConstraints,
		FMath::Abs(FMath::DegreesToRadians(Command.AngularRateDegPerSec) * Command.RadiusCm));
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

FAutopilotIntentHandle UAutopilotComponent::SubmitRootMotionKinematic(
	const FAutopilotKinematicRootMotionCommand& Command)
{
	FAutopilotRootMotionRequest Request;
	Request.Playback = &Command.Playback;
	Request.Intent.Type = EAutopilotMovementIntentType::RootMotion;
	Request.Intent.ArrivalCriteria = Command.ArrivalCriteria;
	Request.Intent.TimeoutSeconds = Command.TimeoutSeconds;
	Request.DriveMode = EAircraftSimulationDriveMode::Kinematic;
	Request.bApplyRootMotionRotation = Command.bApplyRootMotionRotation;
	return SubmitRootMotionRequest(Request);
}

FAutopilotIntentHandle UAutopilotComponent::SubmitRootMotionFlightController(
	const FAutopilotFlightControllerRootMotionCommand& Command)
{
	FAutopilotRootMotionRequest Request;
	Request.Playback = &Command.Playback;
	Request.Intent.Type = EAutopilotMovementIntentType::RootMotion;
	Request.Intent.MotionConstraints = Command.MotionConstraints;
	Request.Intent.ArrivalCriteria = Command.ArrivalCriteria;
	Request.Intent.TimeoutSeconds = Command.TimeoutSeconds;
	Request.DriveMode = EAircraftSimulationDriveMode::FlightController;
	Request.bApplyRootMotionRotation = Command.bApplyRootMotionRotation;
	if (Command.bApplyRootMotionRotation)
	{
		Request.Intent.HeadingMode = EAutopilotHeadingMode::KeepCurrent;
	}
	else
	{
		ApplyHeadingOptions(Request.Intent, Command.Heading);
	}
	return SubmitRootMotionRequest(Request);
}

FAutopilotIntentHandle UAutopilotComponent::SubmitRootMotionPhysicsConstraint(
	const FAutopilotPhysicsConstraintRootMotionCommand& Command)
{
	FAutopilotRootMotionRequest Request;
	Request.Playback = &Command.Playback;
	Request.Intent.Type = EAutopilotMovementIntentType::RootMotion;
	Request.Intent.ArrivalCriteria = Command.ArrivalCriteria;
	Request.Intent.TimeoutSeconds = Command.TimeoutSeconds;
	Request.DriveMode = EAircraftSimulationDriveMode::PhysicsConstraint;
	Request.bApplyRootMotionRotation = Command.bApplyRootMotionRotation;
	if (Command.bApplyRootMotionRotation)
	{
		Request.Intent.HeadingMode = EAutopilotHeadingMode::KeepCurrent;
	}
	else
	{
		ApplyHeadingOptions(Request.Intent, Command.Heading);
	}
	return SubmitRootMotionRequest(Request);
}

FAutopilotIntentHandle UAutopilotComponent::SubmitRootMotionRequest(
	const FAutopilotRootMotionRequest& Request)
{
	if (ActiveRootMotionHandle.IsValid())
	{
		const FVector CurrentLocation = GetOwner()
			? GetOwner()->GetActorLocation() : FVector::ZeroVector;
		FAutopilotVehicleSnapshot RootMotionSnapshot;
		CaptureActiveRootMotionSnapshot(
			RootMotionSnapshot, 0.0f, CurrentLocation);
		MovementExecutor->FinishExternalIntent(
			ActiveRootMotionHandle,
			RootMotionSnapshot,
			EAutopilotIntentStatus::Interrupted,
			EAutopilotIntentFailureReason::Replaced);
		CleanupRootMotionIntent(true);
	}

	const FAutopilotMontagePlayback* Playback = Request.Playback;
	USkeletalMeshComponent* SkeletalMesh = GetOwner()
		? Cast<USkeletalMeshComponent>(GetOwner()->GetRootComponent()) : nullptr;
	UAnimInstance* AnimInstance =
		SkeletalMesh ? SkeletalMesh->GetAnimInstance() : nullptr;
	FAutopilotVehicleSnapshot Snapshot;
	bool bHasControllerState = false;
	if (Request.DriveMode == EAircraftSimulationDriveMode::Kinematic
		&& GetOwner())
	{
		Snapshot = MakeRootMotionSnapshot(
			0.0f, GetOwner()->GetActorLocation());
		bHasControllerState = true;
	}
	else
	{
		bHasControllerState = CaptureSnapshot(Snapshot);
	}
	EAutopilotIntentFailureReason RejectionReason = EAutopilotIntentFailureReason::None;
	if (!FlightController || !bHasControllerState)
	{
		RejectionReason = EAutopilotIntentFailureReason::FlightControllerUnavailable;
	}
	else if (!bAutopilotActive)
	{
		RejectionReason = EAutopilotIntentFailureReason::AutopilotInactive;
	}
	else if (!Playback
		|| !SkeletalMesh
		|| !GetOwner()
		|| !Playback->Montage
		|| !AnimInstance
		|| !Playback->Montage->HasRootMotion()
		|| !FMath::IsFinite(Request.Intent.TimeoutSeconds)
		|| !FMath::IsFinite(Playback->PlayRate)
		|| Playback->PlayRate <= UE_SMALL_NUMBER
		|| !FMath::IsFinite(Playback->StartPositionSeconds)
		|| Playback->StartPositionSeconds < 0.0f
		|| Playback->StartPositionSeconds >= Playback->Montage->GetPlayLength())
	{
		RejectionReason = EAutopilotIntentFailureReason::InvalidIntent;
	}
	else if (!IsRootMotionArrivalFinite(Request.Intent.ArrivalCriteria))
	{
		RejectionReason = EAutopilotIntentFailureReason::InvalidIntent;
	}
	else if (Request.DriveMode == EAircraftSimulationDriveMode::FlightController
		&& !AreRootMotionConstraintsFinite(Request.Intent.MotionConstraints))
	{
		RejectionReason = EAutopilotIntentFailureReason::InvalidIntent;
	}
	const FAutopilotIntentHandle Handle =
		MovementExecutor->Submit(Request.Intent, Snapshot, RejectionReason);
	if (MovementExecutor->GetResult(Handle).Status != EAutopilotIntentStatus::Accepted)
	{
		BroadcastIntentEvents();
		return Handle;
	}

	ActiveRootMotionMesh = SkeletalMesh;
	ActiveRootMotionAnimInstance = AnimInstance;
	ActiveRootMotionMontage = Playback->Montage;
	ActiveRootMotionHandle = Handle;
	ActiveRootMotionDriveMode = Request.DriveMode;
	bActiveRootMotionApplyRotation = Request.bApplyRootMotionRotation;
	bRootMotionMontageEnded = false;
	bRootMotionMontageInterrupted = false;
	ActiveRootMotionStartPositionSeconds = Playback->StartPositionSeconds;
	ActiveRootMotionTargetPositionCm = Snapshot.PositionCm;
	PreviousRootMotionTargetPositionCm = Snapshot.PositionCm;
	ActiveRootMotionTrajectoryActorRotation =
		SkeletalMesh->GetComponentQuat();
	ActiveRootMotionDesiredActorRotation =
		ActiveRootMotionTrajectoryActorRotation;
	PreviousRootMotionDesiredActorRotation =
		ActiveRootMotionTrajectoryActorRotation;
	PreviousRootMotionTargetVelocityCmPerSec = Snapshot.VelocityCmPerSec;
	ActiveRootMotionTargetVelocityCmPerSec = FVector::ZeroVector;
	ActiveRootMotionTargetAccelerationCmPerSecSq = FVector::ZeroVector;
	ActiveRootMotionTargetAngularVelocityWorldDegPerSec = FVector::ZeroVector;
	PreviousRootMotionTargetYawDegrees = Snapshot.YawDegrees;
	RootMotionArrivalStableTimeSeconds = 0.0f;
	if (MotionProfile
		&& ActiveRootMotionDriveMode
			== EAircraftSimulationDriveMode::FlightController)
	{
		MotionProfile->Initialize(
			Snapshot.PositionCm,
			Snapshot.VelocityCmPerSec,
			Snapshot.AccelerationCmPerSecSq,
			Snapshot.YawDegrees,
			0.0f);
	}
	if (ActiveRootMotionDriveMode
		== EAircraftSimulationDriveMode::FlightController)
	{
		ApplyIntentMotionLimits();
	}
	RefreshSimulationDriveSelection();
	if (SimulationBudget.DriveMode != ActiveRootMotionDriveMode)
	{
		MovementExecutor->FinishExternalIntent(
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
		Playback->PlayRate,
		EMontagePlayReturnType::MontageLength,
		Playback->StartPositionSeconds,
		Playback->bStopAllMontages);
	if (MontageDuration <= 0.0f)
	{
		MovementExecutor->FinishExternalIntent(
			Handle,
			Snapshot,
			EAutopilotIntentStatus::Failed,
			EAutopilotIntentFailureReason::InvalidIntent);
		CleanupRootMotionIntent(false);
	}
	else
	{
		FOnMontageEnded EndDelegate;
		EndDelegate.BindUObject(
			this, &ThisClass::HandleRootMotionMontageEnded);
		ActiveRootMotionAnimInstance->Montage_SetEndDelegate(
			EndDelegate, ActiveRootMotionMontage);
		bRootMotionMontageEnded = false;
		bRootMotionMontageInterrupted = false;
	}
	BroadcastIntentEvents();
	return Handle;
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
	ApplyContinuousConstraints(Intent, Command.MotionConstraints,
		FMath::Abs(FMath::DegreesToRadians(Command.AngularRateDegPerSec) * Command.RadiusCm));
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
	ApplyContinuousConstraints(Intent, Command.MotionConstraints,
		FVector2D(Command.DesiredVelocityCmPerSec.X, Command.DesiredVelocityCmPerSec.Y).Size());
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
	const bool bCancellingRootMotion = Handle == ActiveRootMotionHandle;
	FAutopilotVehicleSnapshot Snapshot;
	if (bCancellingRootMotion)
	{
		const FVector CurrentLocation = GetOwner()
			? GetOwner()->GetActorLocation() : FVector::ZeroVector;
		if (!CaptureActiveRootMotionSnapshot(Snapshot, 0.0f, CurrentLocation))
		{
			return false;
		}
		CleanupRootMotionIntent(true);
	}
	else if (!CaptureSnapshot(Snapshot))
	{
		return false;
	}
	const bool bCancelled = MovementExecutor->Cancel(Handle, Snapshot);
	if (bCancelled)
	{
		if (bCancellingRootMotion && MotionProfile)
		{
			MotionProfile->Initialize(
				Snapshot.PositionCm,
				Snapshot.VelocityCmPerSec,
				Snapshot.AccelerationCmPerSecSq,
				Snapshot.YawDegrees,
				0.0f);
		}
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
	if (MovementExecutor
		&& MovementExecutor->GetActiveIntent().Type == EAutopilotMovementIntentType::RootMotion)
	{
		return MovementExecutor->GetCurrentResult().Progress;
	}
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
	FlightController = nullptr;
	FlightControllerComponent = nullptr;
	if (AActor* Owner = GetOwner())
	{
		TArray<UActorComponent*> Components;
		Owner->GetComponents(Components);
		for (UActorComponent* Component : Components)
		{
			if (!Component || !Component->GetClass()->ImplementsInterface(
				UAircraftFlightControllerInterface::StaticClass()))
			{
				continue;
			}
			if (IAircraftFlightControllerInterface* Interface =
				Cast<IAircraftFlightControllerInterface>(Component))
			{
				FlightControllerComponent = Component;
				FlightController.SetObject(Component);
				FlightController.SetInterface(Interface);
				FlightController->SetAircraftAutopilotProvider(this);
				break;
			}
		}
	}
}

void UAutopilotComponent::ApplyProfile()
{
	const UAutopilotProfileAsset* EffectiveProfile = Profile ? Profile : GetDefault<UAutopilotProfileAsset>();
	TurnBehavior->SetLimits(EffectiveProfile->TurnLimits);
	float GravityCmPerSecSq = 980.0f;
	float InitialHoverThrust = 0.5f;
	float VerticalAccelerationMpsSq = 0.0f;
	float CollectiveThrustCommand = 0.0f;
	if (FlightController)
	{
		FlightController->GetAircraftAutopilotPhysicalState(
			GravityCmPerSecSq, InitialHoverThrust,
			VerticalAccelerationMpsSq, CollectiveThrustCommand);
	}
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
		FlightController->GetAircraftAutopilotMotionLimits(
			Requested.CruiseSpeedCmPerSec,
			HardHorizontalSpeed,
			HardHorizontalAcceleration);
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
	if (SimulationBudget.DriveMode == EAircraftSimulationDriveMode::Kinematic
		&& GetOwner())
	{
		OutSnapshot.PositionCm = GetOwner()->GetActorLocation();
		OutSnapshot.VelocityCmPerSec = CachedProfiledSetpoint.bValid
			? CachedProfiledSetpoint.VelocityCmPerSec : FVector::ZeroVector;
		OutSnapshot.AccelerationCmPerSecSq = CachedProfiledSetpoint.bValid
			? CachedProfiledSetpoint.AccelerationCmPerSecSq : FVector::ZeroVector;
		const FQuat ControlWorld = GetOwner()->GetActorQuat()
			* FlightController->GetAircraftControlToBodyRotation();
		OutSnapshot.YawDegrees = ControlWorld.Rotator().Yaw;
		return true;
	}
	FAircraftFlightKinematicState State;
	if (!FlightController->GetAircraftFlightKinematicState(State)) return false;
	OutSnapshot.PositionCm = State.PositionCm;
	OutSnapshot.VelocityCmPerSec = State.VelocityCmPerSec;
	OutSnapshot.AccelerationCmPerSecSq = State.AccelerationWorldCmPerSecSq;
	OutSnapshot.YawDegrees = State.AttitudeDegrees.Yaw;
	return true;
}

void UAutopilotComponent::RefreshSimulationTickEnabled()
{
	const bool bRootMotionRequiresTick = ActiveRootMotionHandle.IsValid()
		&& !SimulationBudget.bIsNetworkProxy;
	const bool bBudgetAllowsTick = bRootMotionRequiresTick
		|| (SimulationBudget.bRunSlowLogic
			&& !SimulationBudget.bIsNetworkProxy);
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
			IAircraftSimulationLODController::
				Execute_RefreshAircraftSimulationDrive(Component);
		}
	}
}

void UAutopilotComponent::TickRootMotionIntent(float DeltaSeconds)
{
	if (!ActiveRootMotionMesh
		|| !ActiveRootMotionAnimInstance
		|| !ActiveRootMotionMontage
		|| !ActiveRootMotionHandle.IsValid()
		|| !GetOwner())
	{
		FAutopilotVehicleSnapshot Snapshot;
		if (GetOwner())
		{
			CaptureActiveRootMotionSnapshot(
				Snapshot, DeltaSeconds, GetOwner()->GetActorLocation());
		}
		MovementExecutor->FinishExternalIntent(
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
		FAutopilotVehicleSnapshot Snapshot;
		CaptureActiveRootMotionSnapshot(
			Snapshot, DeltaSeconds, GetOwner()->GetActorLocation());
		MovementExecutor->FinishExternalIntent(
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

	FAutopilotVehicleSnapshot Snapshot;
	if (!CaptureActiveRootMotionSnapshot(
		Snapshot, DeltaSeconds, PreviousLocation))
	{
		MovementExecutor->FinishExternalIntent(
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
	const float RemainingMontageLength =
		MontageLength - ActiveRootMotionStartPositionSeconds;
	const float Progress = bRootMotionMontageEnded
		? 1.0f
		: (RemainingMontageLength > UE_SMALL_NUMBER
			? FMath::Clamp(
				(MontagePosition - ActiveRootMotionStartPositionSeconds)
					/ RemainingMontageLength,
				0.0f,
				1.0f)
			: 0.0f);
	if (!MovementExecutor->TickExternalIntent(
		ActiveRootMotionHandle, Snapshot, DeltaSeconds, Progress))
	{
		if (MotionProfile)
		{
			MotionProfile->Initialize(
				Snapshot.PositionCm,
				Snapshot.VelocityCmPerSec,
				Snapshot.AccelerationCmPerSecSq,
				Snapshot.YawDegrees,
				0.0f);
		}
		CleanupRootMotionIntent(true);
		InvalidateOutputs();
		BroadcastIntentEvents();
		return;
	}

	if (!bRootMotionMontageEnded
		&& ActiveRootMotionAnimInstance->Montage_GetIsStopped(ActiveRootMotionMontage))
	{
		// A natural end is reported by OnMontageEnded. A stopped montage without
		// that callback must never be promoted to a successful Root Motion intent.
		bRootMotionMontageEnded = true;
		bRootMotionMontageInterrupted = true;
	}
	if (!bRootMotionMontageEnded)
	{
		return;
	}

	const FAutopilotIntentHandle CompletedHandle = ActiveRootMotionHandle;
	const bool bInterrupted = bRootMotionMontageInterrupted;
	if (!bInterrupted
		&& !HasReachedRootMotionTarget(Snapshot, DeltaSeconds))
	{
		return;
	}

	const bool bFlightControllerDriven =
		ActiveRootMotionDriveMode
			== EAircraftSimulationDriveMode::FlightController;
	const FVector FinalTargetPositionCm = ActiveRootMotionTargetPositionCm;
	MovementExecutor->FinishExternalIntent(
		CompletedHandle,
		Snapshot,
		bInterrupted ? EAutopilotIntentStatus::Interrupted : EAutopilotIntentStatus::Succeeded,
		bInterrupted
			? EAutopilotIntentFailureReason::AnimationInterrupted
			: EAutopilotIntentFailureReason::None);
	if (!bInterrupted)
	{
		MovementExecutor->EnterHold(Snapshot, &FinalTargetPositionCm);
	}
	if (MotionProfile && (!bFlightControllerDriven || bInterrupted))
	{
		MotionProfile->Initialize(
			Snapshot.PositionCm,
			Snapshot.VelocityCmPerSec,
			Snapshot.AccelerationCmPerSecSq,
			Snapshot.YawDegrees,
			0.0f);
		if (!bInterrupted)
		{
			CachedProfiledSetpoint = MotionProfile->GetCurrentSetpoint();
			CachedGuidanceCommand = FGuidanceCommand();
			CachedTurnCommand = FTurnCommand();
			CachedFeedForward = FFeedForward();
			if (CachedProfiledSetpoint.bValid)
			{
				FeedForwardCalculator->Compute(
					CachedProfiledSetpoint, CachedFeedForward);
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

void UAutopilotComponent::AccumulateRootMotionTarget(
	const FTransform& WorldRootMotion)
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
		ActiveRootMotionTrajectoryActorRotation
		* ActorLocalRotation).GetNormalized();
}

void UAutopilotComponent::UpdateRootMotionTarget(
	const FAutopilotVehicleSnapshot& Snapshot,
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
		? (ActiveRootMotionTargetPositionCm
			- PreviousRootMotionTargetPositionCm) / DeltaSeconds
		: FVector::ZeroVector;
	ActiveRootMotionTargetAccelerationCmPerSecSq =
		(ActiveRootMotionTargetVelocityCmPerSec
			- PreviousRootMotionTargetVelocityCmPerSec) / DeltaSeconds;
	NominalSetpoint.VelocityCmPerSec =
		ActiveRootMotionTargetVelocityCmPerSec;
	NominalSetpoint.AccelerationCmPerSecSq =
		ActiveRootMotionTargetAccelerationCmPerSecSq;

	const FQuat ControlToBody = FlightController
		? FlightController->GetAircraftControlToBodyRotation()
		: FQuat::Identity;
	const float RootMotionYawDegrees = (
		ActiveRootMotionTrajectoryActorRotation * ControlToBody).Rotator().Yaw;
	NominalSetpoint.YawDegrees = PreviousRootMotionTargetYawDegrees;
	if (bActiveRootMotionApplyRotation)
	{
		NominalSetpoint.YawDegrees = RootMotionYawDegrees;
		ActiveRootMotionDesiredActorRotation =
			ActiveRootMotionTrajectoryActorRotation;
		NominalSetpoint.YawRateDegreesPerSec = bConsumedRootMotion
			? FMath::FindDeltaAngleDegrees(
				PreviousRootMotionTargetYawDegrees, RootMotionYawDegrees)
				/ DeltaSeconds
			: 0.0f;
	}
	else
	{
		MovementExecutor->ApplyHeading(Snapshot, NominalSetpoint);
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
				RotationAxis.GetSafeNormal()
				* (RotationAngleRadians / DeltaSeconds));

	PreviousRootMotionTargetPositionCm = ActiveRootMotionTargetPositionCm;
	PreviousRootMotionTargetVelocityCmPerSec =
		ActiveRootMotionTargetVelocityCmPerSec;
	PreviousRootMotionTargetYawDegrees = NominalSetpoint.YawDegrees;
	PreviousRootMotionDesiredActorRotation =
		ActiveRootMotionDesiredActorRotation;

	if (ActiveRootMotionDriveMode
		!= EAircraftSimulationDriveMode::FlightController)
	{
		InvalidateOutputs();
		return;
	}
	if (!MotionProfile)
	{
		InvalidateOutputs();
		return;
	}
	ApplyIntentMotionLimits();

	CachedGuidanceCommand = FGuidanceCommand();
	CachedTurnCommand = FTurnCommand();
	const UAutopilotProfileAsset* EffectiveProfile =
		Profile ? Profile : GetDefault<UAutopilotProfileAsset>();
	if (EffectiveProfile->bEnableCoordinatedTurns && TurnBehavior)
	{
		CachedTurnCommand = TurnBehavior->Compute(
			NominalSetpoint.VelocityCmPerSec,
			Snapshot.VelocityCmPerSec,
			Snapshot.YawDegrees,
			MovementExecutor->GetActiveIntent().MotionConstraints.MaxYawRateDegPerSec,
			DeltaSeconds);
	}

	CachedProfiledSetpoint = MotionProfile->Update(NominalSetpoint, DeltaSeconds);
	UpdateHoverThrustEstimate(DeltaSeconds);
	CachedFeedForward = FFeedForward();
	if (CachedProfiledSetpoint.bValid)
	{
		FeedForwardCalculator->Compute(CachedProfiledSetpoint, CachedFeedForward);
	}
}

bool UAutopilotComponent::HasReachedRootMotionTarget(
	const FAutopilotVehicleSnapshot& Snapshot,
	float DeltaSeconds)
{
	const FAutopilotArrivalCriteria& Criteria =
		MovementExecutor->GetActiveIntent().ArrivalCriteria;
	const FVector PositionError =
		ActiveRootMotionTargetPositionCm - Snapshot.PositionCm;
	const bool bUsesMotionProfile =
		ActiveRootMotionDriveMode
			== EAircraftSimulationDriveMode::FlightController;
	const FVector SetpointError = bUsesMotionProfile && CachedProfiledSetpoint.bValid
		? ActiveRootMotionTargetPositionCm - CachedProfiledSetpoint.PositionCm
		: PositionError;
	const bool bPositionReached =
		FVector2D(PositionError.X, PositionError.Y).Size()
			<= Criteria.HorizontalToleranceCm
		&& FMath::Abs(PositionError.Z) <= Criteria.VerticalToleranceCm;
	const bool bSetpointReached =
		FVector2D(SetpointError.X, SetpointError.Y).Size()
			<= Criteria.HorizontalToleranceCm
		&& FMath::Abs(SetpointError.Z) <= Criteria.VerticalToleranceCm;
	const bool bSpeedReached =
		Snapshot.VelocityCmPerSec.Size() <= Criteria.SpeedToleranceCmPerSec;
	const FQuat ControlToBody = FlightController
		? FlightController->GetAircraftControlToBodyRotation()
		: FQuat::Identity;
	const float ConstraintTargetYawDegrees = (
		ActiveRootMotionDesiredActorRotation * ControlToBody).Rotator().Yaw;
	const float DesiredYawDegrees =
		bUsesMotionProfile && CachedProfiledSetpoint.bValid
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
				ActiveRootMotionAnimInstance->Montage_GetEndedDelegate(
					ActiveRootMotionMontage))
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
	ActiveRootMotionDriveMode =
		EAircraftSimulationDriveMode::FlightController;
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
	ActiveRootMotionTargetAngularVelocityWorldDegPerSec =
		FVector::ZeroVector;
	PreviousRootMotionTargetYawDegrees = 0.0f;
	RootMotionArrivalStableTimeSeconds = 0.0f;
	ActiveRootMotionStartPositionSeconds = 0.0f;
	RefreshSimulationDriveSelection();
	RefreshSimulationTickEnabled();
}

bool UAutopilotComponent::CaptureActiveRootMotionSnapshot(
	FAutopilotVehicleSnapshot& OutSnapshot,
	float DeltaSeconds,
	const FVector& PreviousLocation) const
{
	if (ActiveRootMotionDriveMode
		!= EAircraftSimulationDriveMode::Kinematic)
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

FAutopilotVehicleSnapshot UAutopilotComponent::MakeRootMotionSnapshot(
	float DeltaSeconds, const FVector& PreviousLocation) const
{
	FAutopilotVehicleSnapshot Snapshot;
	const AActor* Owner = GetOwner();
	if (!Owner)
	{
		return Snapshot;
	}
	Snapshot.PositionCm = Owner->GetActorLocation();
	Snapshot.VelocityCmPerSec = DeltaSeconds > UE_SMALL_NUMBER
		? (Snapshot.PositionCm - PreviousLocation) / DeltaSeconds
		: FVector::ZeroVector;
	const FQuat ControlWorld = FlightController
		? Owner->GetActorQuat() * FlightController->GetAircraftControlToBodyRotation()
		: Owner->GetActorQuat();
	Snapshot.YawDegrees = ControlWorld.Rotator().Yaw;
	return Snapshot;
}

void UAutopilotComponent::HandleRootMotionMontageEnded(
	UAnimMontage* Montage, bool bInterrupted)
{
	if (Montage != ActiveRootMotionMontage)
	{
		return;
	}
	// 只记录结束状态；最后一段 Root Motion 会在 Mesh Tick 完成后的 Autopilot Tick 中消费。
	bRootMotionMontageEnded = true;
	bRootMotionMontageInterrupted = bInterrupted;
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
	OutInjection.YawRateSetpointDegPerSec = CachedFeedForward.YawRateFFDegPerSec
		+ CachedTurnCommand.DesiredYawRateDegPerSec;
	OutInjection.YawRateLimitDegPerSec = MovementExecutor
		? MovementExecutor->GetActiveIntent().MotionConstraints.MaxYawRateDegPerSec : 0.0f;
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
	float GravityCmPerSecSq = 980.0f;
	float HoverThrust = 0.5f;
	float AccelerationMpsSq = 0.0f;
	float CollectiveThrust = 0.0f;
	FlightController->GetAircraftAutopilotPhysicalState(
		GravityCmPerSecSq, HoverThrust, AccelerationMpsSq, CollectiveThrust);
	if (!EffectiveProfile->bEnableHoverThrustEstimator)
	{
		FeedForwardCalculator->SetPhysicalReference(GravityCmPerSecSq, HoverThrust);
		return;
	}
	HoverThrustEstimator.Update(
		DeltaSeconds, AccelerationMpsSq, CollectiveThrust, GravityCmPerSecSq * 0.01f);
	HoverThrust = HoverThrustEstimator.GetHoverThrust();
	FeedForwardCalculator->SetPhysicalReference(GravityCmPerSecSq, HoverThrust);
}

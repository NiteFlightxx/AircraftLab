// 对应 NxGame AircraftAutopilot/Private/AutopilotComponent.cpp（管线编排对齐重写）。
// Root Motion 桥接保持 NxGame 语义：消费 Montage 根位移 → 发布 FAircraftMotionTarget
// （经 IAircraftSimulationLODConsumer）+ 按命令类型发布驱动覆盖。

#include "AircraftRuntimeCommon/Autopilot/AutopilotComponent.h"

#include "Animation/AnimInstance.h"
#include "Animation/AnimMontage.h"
#include "Components/SkeletalMeshComponent.h"
#include "Engine/World.h"
#include "GameFramework/Actor.h"

#include UE_INLINE_GENERATED_CPP_BY_NAME(AutopilotComponent)

DEFINE_LOG_CATEGORY_STATIC(LogAircraftAutopilot, Log, All);

UAutopilotComponent::UAutopilotComponent()
{
	PrimaryComponentTick.bCanEverTick = true;
	PrimaryComponentTick.TickGroup = TG_PrePhysics;
	PrimaryComponentTick.bStartWithTickEnabled = false;
}

void UAutopilotComponent::OnRegister()
{
	Super::OnRegister();
	CreateRuntimeObjects();
	ResolveFlightController();
}

void UAutopilotComponent::BeginPlay()
{
	Super::BeginPlay();
	ResolveFlightController();
	ResolveAutopilotConfig(/*bForceRefresh=*/true);
}

void UAutopilotComponent::EndPlay(const EEndPlayReason::Type EndPlayReason)
{
	CleanupRootMotionIntent(/*bStopMontage=*/true);
	if (bAutopilotActive)
	{
		SetAutopilotActive(false);
	}
	Super::EndPlay(EndPlayReason);
}

void UAutopilotComponent::CreateRuntimeObjects()
{
	MovementExecutor.Initialize();
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
	TurnBehavior.SetGravity(GravityCmPerSecSq);
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
	if (bAutopilotActive == bActive)
	{
		return;
	}

	if (bActive)
	{
		ResolveFlightController();
		if (!FlightController.GetInterface())
		{
			UE_LOG(LogAircraftAutopilot, Warning,
				TEXT("Autopilot on '%s' cannot activate: no flight controller interface found."),
				*GetNameSafe(GetOwner()));
			return;
		}
		ResolveAutopilotConfig(/*bForceRefresh=*/true);
		FlightModeBeforeActivation = FlightController->ActivateAircraftAutopilotControl();
		bFlightModeBeforeActivationCaptured = true;
		bAutopilotActive = true;
		UpdateTickEnabled();

		FAircraftAutopilotVehicleSnapshot Snapshot;
		if (CaptureSnapshot(Snapshot))
		{
			MotionProfile.Initialize(
				Snapshot.PositionCm, Snapshot.VelocityCmPerSec, Snapshot.AccelerationCmPerSecSq,
				Snapshot.YawDegrees, 0.0f);
			MovementExecutor.EnterHold(Snapshot);
			ApplyIntentMotionLimits();
		}
	}
	else
	{
		bAutopilotActive = false;
		CleanupRootMotionIntent(/*bStopMontage=*/true);
		if (FlightController.GetInterface() && bFlightModeBeforeActivationCaptured)
		{
			FlightController->DeactivateAircraftAutopilotControl(FlightModeBeforeActivation);
		}
		bFlightModeBeforeActivationCaptured = false;
		MovementExecutor.CancelActive(EAutopilotIntentFailureReason::AutopilotInactive);
		InvalidateOutputs();
		UpdateTickEnabled();
	}
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
	CachedInjection = FAutopilotInjection();
}

void UAutopilotComponent::TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction)
{
	Super::TickComponent(DeltaTime, TickType, ThisTickFunction);

	// Root Motion 意图有独立的驱动路径（不依赖激活状态外的轨迹管线）
	TickRootMotionIntent(DeltaTime);

	if (!bAutopilotActive)
	{
		return;
	}

	if (!FlightController.GetInterface())
	{
		ResolveFlightController();
		if (!FlightController.GetInterface())
		{
			SetAutopilotActive(false);
			return;
		}
	}
	// 资产热重载后配置可能失效
	ResolveAutopilotConfig(/*bForceRefresh=*/false);

	FAircraftAutopilotVehicleSnapshot Snapshot;
	if (!CaptureSnapshot(Snapshot))
	{
		return;
	}

	/* 1. 名义设定值（轨迹生成） */
	FTrajectoryPoint NominalSetpoint;
	if (!MovementExecutor.BuildSetpoint(Snapshot, DeltaTime, CachedProfiledSetpoint, NominalSetpoint))
	{
		BuildInjection();
		BroadcastIntentEvents();
		return;
	}

	/* 2. 路径制导（仅路径类轨迹；策略=Direct 或无轨迹时跳过） */
	CachedGuidanceCommand = FGuidanceCommand();
	if (PathFollowing.IsValid()
		&& MovementExecutor.GetTrajectoryGenerator()->IsValid()
		&& (MovementExecutor.GetActiveIntent().Type == EAutopilotMovementIntentType::FollowPath
			|| MovementExecutor.GetActiveIntent().Type == EAutopilotMovementIntentType::Orbit
			|| MovementExecutor.GetActiveIntent().Type == EAutopilotMovementIntentType::CircleArc))
	{
		FGuidanceCommand Guidance;
		if (PathFollowing->Update(Snapshot.PositionCm, Snapshot.VelocityCmPerSec, DeltaTime, Guidance)
			&& Guidance.bValid)
		{
			CachedGuidanceCommand = Guidance;
			// 制导律改写水平速度方向（幅值与垂直分量保留名义剖面）
			NominalSetpoint.VelocityCmPerSec = Guidance.DesiredVelocityCmPerSec;
			NominalSetpoint.YawDegrees = Guidance.DesiredYawDegrees;
			NominalSetpoint.YawRateDegreesPerSec = Guidance.DesiredYawRateDegPerSec;
		}
	}

	/* 3. 航向应用（意图的 HeadingMode 覆盖几何航向） */
	MovementExecutor.ApplyHeading(Snapshot, NominalSetpoint);

	/* 4. 协调转弯（滚转 + 偏航角速度前馈） */
	CachedTurnCommand = FTurnCommand();
	if (AutopilotConfig.bEnableCoordinatedTurns)
	{
		float GravityCmPerSecSq = 980.0f;
		float HoverCollective = 0.5f;
		float VerticalAccelMpsSq = 0.0f;
		float CollectiveCommand = 0.0f;
		FlightController->GetAircraftAutopilotPhysicalState(
			GravityCmPerSecSq, HoverCollective, VerticalAccelMpsSq, CollectiveCommand);
		TurnBehavior.SetGravity(GravityCmPerSecSq);
		CachedTurnCommand = TurnBehavior.Compute(
			NominalSetpoint.VelocityCmPerSec, Snapshot.VelocityCmPerSec,
			Snapshot.YawDegrees, MovementExecutor.GetActiveIntent().MotionConstraints.MaxYawRateDegPerSec,
			DeltaTime);
	}

	/* 5. Motion Profile 整形（物理可达） */
	CachedProfiledSetpoint = MotionProfile.Update(NominalSetpoint, DeltaTime);

	/* 6. 悬停油门 EKF + 前馈 */
	UpdateHoverThrustEstimate(DeltaTime);
	FeedForwardCalculator.SetPhysicalReference(
		GetWorld() ? -GetWorld()->GetGravityZ() : 980.0f,
		AutopilotConfig.bEnableHoverThrustEstimator && HoverThrustEstimator.IsInitialized()
			? HoverThrustEstimator.GetHoverThrust()
			: 0.5f);
	{
		float GravityCmPerSecSq = 980.0f;
		float HoverCollective = 0.5f;
		float VerticalAccelMpsSq = 0.0f;
		float CollectiveCommand = 0.0f;
		FlightController->GetAircraftAutopilotPhysicalState(
			GravityCmPerSecSq, HoverCollective, VerticalAccelMpsSq, CollectiveCommand);
		FeedForwardCalculator.SetPhysicalReference(
			GravityCmPerSecSq,
			AutopilotConfig.bEnableHoverThrustEstimator && HoverThrustEstimator.IsInitialized()
				? HoverThrustEstimator.GetHoverThrust()
				: HoverCollective);
	}
	FeedForwardCalculator.Compute(CachedProfiledSetpoint, CachedFeedForward);

	/* 7. 完成判定 + 事件 + 注入缓存 */
	MovementExecutor.UpdateCompletion(Snapshot, DeltaTime, CachedProfiledSetpoint);
	BuildInjection();
	BroadcastIntentEvents();
}

void UAutopilotComponent::UpdateHoverThrustEstimate(float DeltaSeconds)
{
	if (!AutopilotConfig.bEnableHoverThrustEstimator || !FlightController.GetInterface())
	{
		return;
	}
	float GravityCmPerSecSq = 980.0f;
	float HoverCollective = 0.5f;
	float VerticalAccelMpsSq = 0.0f;
	float CollectiveCommand = 0.0f;
	FlightController->GetAircraftAutopilotPhysicalState(
		GravityCmPerSecSq, HoverCollective, VerticalAccelMpsSq, CollectiveCommand);
	HoverThrustEstimator.Update(
		DeltaSeconds, VerticalAccelMpsSq, CollectiveCommand, GravityCmPerSecSq * 0.01f);
}

void UAutopilotComponent::BuildInjection()
{
	CachedInjection = FAutopilotInjection();
	if (!CachedProfiledSetpoint.bValid)
	{
		return;
	}

	CachedInjection.PositionSetpointCm = CachedProfiledSetpoint.PositionCm;
	CachedInjection.VelocitySetpointCmPerSec = CachedFeedForward.VelocityFFCmPerSec;
	CachedInjection.AccelerationSetpointCmPerSecSq = CachedFeedForward.AccelFFCmPerSecSq;
	CachedInjection.AltitudeSetpointCm = static_cast<float>(CachedProfiledSetpoint.PositionCm.Z);
	CachedInjection.VerticalVelocitySetpointCmPerSec = static_cast<float>(CachedProfiledSetpoint.VelocityCmPerSec.Z);
	CachedInjection.ThrustFeedForward = CachedFeedForward.ThrustFF;
	CachedInjection.YawSetpointDegrees = CachedProfiledSetpoint.YawDegrees;
	CachedInjection.YawRateSetpointDegPerSec = CachedFeedForward.YawRateFFDegPerSec;
	CachedInjection.YawRateLimitDegPerSec =
		MovementExecutor.GetActiveIntent().MotionConstraints.MaxYawRateDegPerSec;
	CachedInjection.TurnRollDegrees = CachedTurnCommand.bValid && CachedTurnCommand.bCoordinatedTurn
		? CachedTurnCommand.DesiredRollDegrees : 0.0f;
	CachedInjection.bValid = true;
}

bool UAutopilotComponent::GetAutopilotInjection(FAutopilotInjection& OutInjection) const
{
	if (!bAutopilotActive || !CachedInjection.bValid)
	{
		OutInjection = FAutopilotInjection();
		return false;
	}
	OutInjection = CachedInjection;
	return true;
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
	Intent.FixedYawDegrees = Heading.FixedYawDegrees;
	Intent.DesiredYawRateDegPerSec = Heading.YawRateDegreesPerSec;
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
	// 持续命令没有"终点制动"，巡航速度由期望速度/半径×角速度唯一决定
	Intent.MotionConstraints.CruiseSpeedCmPerSec = CommandedHorizontalSpeedCmPerSec;
	Intent.MotionConstraints.MaxAccelerationCmPerSecSq = Constraints.MaxAccelerationCmPerSecSq;
	Intent.MotionConstraints.MaxJerkCmPerSecCubed = Constraints.MaxJerkCmPerSecCubed;
	Intent.MotionConstraints.MaxClimbRateCmPerSec = Constraints.MaxClimbRateCmPerSec;
	Intent.MotionConstraints.MaxDescentRateCmPerSec = Constraints.MaxDescentRateCmPerSec;
	Intent.MotionConstraints.MaxVerticalAccelerationCmPerSecSq = Constraints.MaxVerticalAccelerationCmPerSecSq;
	Intent.MotionConstraints.MaxVerticalJerkCmPerSecCubed = Constraints.MaxVerticalJerkCmPerSecCubed;
	Intent.MotionConstraints.MaxYawRateDegPerSec = Constraints.MaxYawRateDegPerSec;
	Intent.MotionConstraints.MaxYawAccelerationDegPerSecSq = Constraints.MaxYawAccelerationDegPerSecSq;
	Intent.MotionConstraints.MaxYawJerkDegPerSecCubed = Constraints.MaxYawJerkDegPerSecCubed;
}

FAutopilotIntentHandle UAutopilotComponent::SubmitMovementIntent(const FAutopilotMovementIntent& Intent)
{
	FAircraftAutopilotVehicleSnapshot Snapshot;
	EAutopilotIntentFailureReason Rejection = EAutopilotIntentFailureReason::None;
	if (!bAutopilotActive)
	{
		Rejection = EAutopilotIntentFailureReason::AutopilotInactive;
	}
	else if (!CaptureSnapshot(Snapshot))
	{
		Rejection = EAutopilotIntentFailureReason::FlightControllerUnavailable;
	}
	const FAutopilotIntentHandle Handle = MovementExecutor.Submit(Intent, Snapshot, Rejection);
	if (Handle.IsValid())
	{
		ApplyIntentMotionLimits();
	}
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
	if (!Playback.Montage)
	{
		return false;
	}

	AActor* const OwnerActor = GetOwner();
	USkeletalMeshComponent* Mesh = OwnerActor ? OwnerActor->FindComponentByClass<USkeletalMeshComponent>() : nullptr;
	if (!Mesh)
	{
		return false;
	}
	UAnimInstance* AnimInstance = Mesh->GetAnimInstance();
	if (!AnimInstance)
	{
		return false;
	}

	// bStopAllMontages 由 Montage_Play 的末参直接承担
	const float Duration = AnimInstance->Montage_Play(
		Playback.Montage, Playback.PlayRate, EMontagePlayReturnType::MontageLength,
		Playback.StartPositionSeconds, Playback.bStopAllMontages);
	if (Duration <= 0.0f)
	{
		return false;
	}

	// 无 Root Motion 的普通 Montage 仅播放动画
	if (!Playback.Montage->HasRootMotion())
	{
		return true;
	}

	// 带 Root Motion：按当前驱动模式创建对应 Intent
	FAutopilotMovementIntent Intent;
	Intent.Type = EAutopilotMovementIntentType::RootMotion;
	Intent.TimeoutSeconds = Duration / FMath::Max(Playback.PlayRate, UE_SMALL_NUMBER) + 2.0f;
	OutRootMotionHandle = SubmitMovementIntent(Intent);
	if (OutRootMotionHandle.IsValid())
	{
		ActiveRootMotionMesh = Mesh;
		ActiveRootMotionAnimInstance = AnimInstance;
		ActiveRootMotionMontage = Playback.Montage;
		ActiveRootMotionHandle = OutRootMotionHandle;
		ActiveRootMotionStartPositionSeconds = Playback.StartPositionSeconds;
		bRootMotionMontageEnded = false;
		AnimInstance->OnMontageEnded.AddUniqueDynamic(this, &UAutopilotComponent::HandleRootMotionMontageEnded);
		UpdateTickEnabled();
	}
	else
	{
		AnimInstance->Montage_Stop(0.0f, Playback.Montage);
		return false;
	}
	return true;
}

FAutopilotIntentHandle UAutopilotComponent::SubmitRootMotionKinematic(const FAutopilotKinematicRootMotionCommand& Command)
{
	FAutopilotIntentHandle Handle;
	if (PlayMontage(Command.Playback, Handle) && Handle.IsValid())
	{
		ActiveRootMotionDriveOverride.DriveMode = EAircraftSimulationDriveMode::Kinematic;
		ActiveRootMotionDriveOverride.Priority = 100;
		ActiveRootMotionDriveOverride.bValid = true;
	}
	return Handle;
}

FAutopilotIntentHandle UAutopilotComponent::SubmitRootMotionFlightController(const FAutopilotFlightControllerRootMotionCommand& Command)
{
	FAutopilotIntentHandle Handle;
	if (PlayMontage(Command.Playback, Handle) && Handle.IsValid())
	{
		ActiveRootMotionDriveOverride.DriveMode = EAircraftSimulationDriveMode::FlightController;
		ActiveRootMotionDriveOverride.Priority = 100;
		ActiveRootMotionDriveOverride.bValid = true;
	}
	return Handle;
}

FAutopilotIntentHandle UAutopilotComponent::SubmitRootMotionPhysicsConstraint(const FAutopilotPhysicsConstraintRootMotionCommand& Command)
{
	FAutopilotIntentHandle Handle;
	if (PlayMontage(Command.Playback, Handle) && Handle.IsValid())
	{
		ActiveRootMotionDriveOverride.DriveMode = EAircraftSimulationDriveMode::PhysicsConstraint;
		ActiveRootMotionDriveOverride.Priority = 100;
		ActiveRootMotionDriveOverride.bValid = true;
	}
	return Handle;
}

void UAutopilotComponent::TickRootMotionIntent(float DeltaSeconds)
{
	if (!ActiveRootMotionHandle.IsValid())
	{
		return;
	}

	FAircraftAutopilotVehicleSnapshot Snapshot;
	if (!CaptureSnapshot(Snapshot))
	{
		return;
	}

	// 消费 Montage 根位移增量（组件空间 → 世界系）→ 累积运动目标：
	// 目标位置 = 当前位置 + 本帧根位移增量；速度 = 增量 / Δt。
	bool bConsumed = false;
	if (IsValid(ActiveRootMotionMesh) && IsValid(ActiveRootMotionAnimInstance) && !bRootMotionMontageEnded)
	{
		const FRootMotionMovementParams RootMotionParams =
			ActiveRootMotionAnimInstance->ConsumeExtractedRootMotion(1.0f);
		const FVector LocalDelta = RootMotionParams.GetRootMotionTransform().GetTranslation();
		if (!LocalDelta.IsNearlyZero())
		{
			const FVector WorldDelta =
				ActiveRootMotionMesh->GetComponentQuat().RotateVector(LocalDelta);
			ActiveRootMotionTarget.PositionCm = Snapshot.PositionCm + WorldDelta;
			ActiveRootMotionTarget.VelocityCmPerSec = WorldDelta / FMath::Max(DeltaSeconds, UE_SMALL_NUMBER);
			ActiveRootMotionTarget.RotationDegrees = FRotator(0.0f, Snapshot.YawDegrees, 0.0f);
			ActiveRootMotionTarget.Priority = 100;
			ActiveRootMotionTarget.bValid = true;
			bConsumed = true;
		}
	}

	// 完成判定：Montage 结束且已到位
	const float Progress = IsValid(ActiveRootMotionMontage) && IsValid(ActiveRootMotionAnimInstance)
		? FMath::Clamp(ActiveRootMotionAnimInstance->Montage_GetPosition(ActiveRootMotionMontage)
			/ FMath::Max(ActiveRootMotionMontage->GetPlayLength(), UE_SMALL_NUMBER), 0.0f, 1.0f)
		: 1.0f;
	MovementExecutor.TickExternalIntent(ActiveRootMotionHandle, Snapshot, DeltaSeconds, bConsumed ? Progress : 1.0f);

	if (bRootMotionMontageEnded)
	{
		MovementExecutor.FinishExternalIntent(ActiveRootMotionHandle, Snapshot,
			EAutopilotIntentStatus::Succeeded, EAutopilotIntentFailureReason::None);
		CleanupRootMotionIntent(/*bStopMontage=*/false);
	}
}

void UAutopilotComponent::CleanupRootMotionIntent(bool bStopMontage)
{
	if (IsValid(ActiveRootMotionAnimInstance))
	{
		ActiveRootMotionAnimInstance->OnMontageEnded.RemoveAll(this);
		if (bStopMontage && IsValid(ActiveRootMotionMontage))
		{
			ActiveRootMotionAnimInstance->Montage_Stop(0.2f, ActiveRootMotionMontage);
		}
	}
	ActiveRootMotionMesh = nullptr;
	ActiveRootMotionAnimInstance = nullptr;
	ActiveRootMotionMontage = nullptr;
	ActiveRootMotionHandle = FAutopilotIntentHandle();
	ActiveRootMotionTarget = FAircraftMotionTarget();
	ActiveRootMotionDriveOverride = FAircraftSimulationDriveOverride();
	UpdateTickEnabled();
}

void UAutopilotComponent::UpdateTickEnabled()
{
	const bool bBudgetAllowsTick = SimulationBudget.LODIndex == INDEX_NONE
		|| (SimulationBudget.bRunSlowLogic && !SimulationBudget.bIsNetworkProxy);
	SetComponentTickEnabled(bBudgetAllowsTick
		&& (bAutopilotActive || ActiveRootMotionHandle.IsValid()));
}

void UAutopilotComponent::HandleRootMotionMontageEnded(UAnimMontage* Montage, bool bInterrupted)
{
	if (Montage == ActiveRootMotionMontage)
	{
		bRootMotionMontageEnded = true;
		if (bInterrupted && ActiveRootMotionHandle.IsValid())
		{
			FAircraftAutopilotVehicleSnapshot Snapshot;
			CaptureSnapshot(Snapshot);
			MovementExecutor.FinishExternalIntent(ActiveRootMotionHandle, Snapshot,
				EAutopilotIntentStatus::Interrupted, EAutopilotIntentFailureReason::AnimationInterrupted);
			CleanupRootMotionIntent(/*bStopMontage=*/false);
		}
	}
}

/* ---------------------------------------------------------------------------
 * IAircraftSimulationLODConsumer
 * ------------------------------------------------------------------------- */

void UAutopilotComponent::ApplyAircraftSimulationBudget_Implementation(const FAircraftSimulationBudget& Budget)
{
	SimulationBudget = Budget;
	SetComponentTickInterval(FMath::Max(Budget.SlowLogicIntervalSeconds, 0.0f));
	ResolveAutopilotConfig(/*bForceRefresh=*/true);
	UpdateTickEnabled();
}

bool UAutopilotComponent::GetAircraftMotionTarget_Implementation(FAircraftMotionTarget& OutTarget) const
{
	if (ActiveRootMotionTarget.bValid)
	{
		OutTarget = ActiveRootMotionTarget;
		return true;
	}
	OutTarget = FAircraftMotionTarget();
	return false;
}

FAircraftSimulationDriveOverride UAutopilotComponent::GetAircraftSimulationDriveOverride_Implementation() const
{
	return ActiveRootMotionDriveOverride;
}

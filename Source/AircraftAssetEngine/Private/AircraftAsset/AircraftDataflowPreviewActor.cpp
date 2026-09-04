#include "AircraftAsset/AircraftDataflowPreviewActor.h"

#include "Animation/AnimationAsset.h"
#include "Engine/SkeletalMesh.h"
#include "AircraftAsset/AircraftAsset.h"   
#include "AircraftAsset/AircraftComponent.h"
#include "AircraftAsset/AircraftSimulationModel.h"
#include "Aircraft/AircraftPhysicsUnits.h"

#include UE_INLINE_GENERATED_CPP_BY_NAME(AircraftDataflowPreviewActor)

AAircraftDataflowPreviewActor::AAircraftDataflowPreviewActor(const FObjectInitializer& ObjectInitializer)
	: Super(ObjectInitializer)
{
	AircraftComponent = CreateDefaultSubobject<UAircraftComponent>(TEXT("AircraftComponent0"));
	// Dataflow deferred spawning registers this native component before it injects the
	// Aircraft asset. Enable physics collision up front so the asset-bound component
	// lifecycle creates every body in the injected PhysicsAsset.
	AircraftComponent->SetCollisionEnabled(ECollisionEnabled::QueryAndPhysics);
	RootComponent = AircraftComponent;
	PrimaryActorTick.bCanEverTick = true;
	PrimaryActorTick.bStartWithTickEnabled = true;
	PreviewMovementIntentHandle.Id = 1;
}

void AAircraftDataflowPreviewActor::OnConstruction(const FTransform& Transform)
{
	Super::OnConstruction(Transform);

	// 引擎 spawn 路径（UE::Dataflow::SpawnSimulatedActor）以 bDeferConstruction=true 生成，
	// SetActorProperties 覆写完三个注入属性之后才 FinishSpawning → 到达这里时
	// DataflowAsset / SkeletalMesh / AnimationAsset 均已就位。
	SyncComponentFromInjectedProperties();
}

void AAircraftDataflowPreviewActor::Tick(float DeltaSeconds)
{
	Super::Tick(DeltaSeconds);
	InitializeDefaultScenarioIfReady();
}

#if WITH_EDITOR
void AAircraftDataflowPreviewActor::PostEditChangeProperty(FPropertyChangedEvent& PropertyChangedEvent)
{
	Super::PostEditChangeProperty(PropertyChangedEvent);
	SyncComponentFromInjectedProperties();
}
#endif

void AAircraftDataflowPreviewActor::SyncComponentFromInjectedProperties()
{
	if (!AircraftComponent)
	{
		return;
	}

	// 1) 资产绑定：UAircraftComponent::SetAsset 内部会同步骨骼网格/物理资产/仿真模型。
	if (AircraftComponent->GetAsset() != DataflowAsset)
	{
		bDefaultScenarioInitialized = false;
		bPreviewIntentActive = false;
		++PreviewMovementIntentRevision;
		AircraftComponent->SetAsset(DataflowAsset);
	}
	AircraftComponent->SetMovementIntentProvider(this);

	// 注入的 SkeletalMesh 只镜像 Dataflow Content。模拟资源始终由 Aircraft Asset 唯一决定。
	bPreviewMeshMismatch = SkeletalMesh && DataflowAsset
		&& SkeletalMesh != DataflowAsset->GetSimulationSkeletalMesh();
	// 预览动画（无人机通常不播动画，但保持与布料预览一致的通路）。
	if (AnimationAsset && AircraftComponent->AnimationData.AnimToPlay != AnimationAsset)
	{
		AircraftComponent->SetAnimationMode(EAnimationMode::Type::AnimationSingleNode);
		AircraftComponent->SetAnimation(AnimationAsset);
	}

	// Deferred Dataflow spawning registers the native component before injecting the
	// Aircraft asset. The assetless registration may leave its primary tick disabled;
	// the preview playback state, rather than that temporary registration state, is
	// authoritative here. A paused scene is frozen immediately by the native manager.
	AircraftComponent->SetComponentTickEnabled(bPreviewPlaybackEnabled);
}

void AAircraftDataflowPreviewActor::InitializeDefaultScenarioIfReady()
{
	if (bDefaultScenarioInitialized || !AircraftComponent)
	{
		return;
	}
	const FAircraftSimulationBackendStatus Status = AircraftComponent->GetSimulationBackendStatus();
	if (Status.State != EAircraftSimulationBackendState::Ready)
	{
		return;
	}

	if (const FAircraftSimulationLodModel* const Model = AircraftComponent->GetCurrentLodModel())
	{
		const FQuat BodyRotation = Model->FlightController.FrameBinding.GetBodyWorldTransform(
			AircraftComponent->GetComponentTransform()).GetRotation();
		PreviewFixedYawDegrees = Model->FlightController.GetControlWorldRotation(
			BodyRotation).Rotator().Yaw;
	}
	PreviewMovementIntent = FAircraftMovementIntent();
	PreviewMovementIntent.Type = EAircraftMovementIntentType::Hold;
	PreviewMovementIntent.Hold.PositionCm = PreviewHoldTargetCm;
	PreviewMovementIntent.Hold.bCaptureCurrentPosition = false;
	PreviewMovementIntent.bHasRequestedMotionLimits = false;
	PreviewMovementIntent.Heading.Mode = EAircraftHeadingMode::FixedYaw;
	PreviewMovementIntent.Heading.FixedYawDegrees = PreviewFixedYawDegrees;
	PreviewMovementIntent.TimeoutSeconds = 0.0f;
	++PreviewMovementIntentRevision;
	bPreviewIntentActive = true;
	bDefaultScenarioInitialized = true;
	AircraftComponent->SetControllerEnabled(true);
	AircraftComponent->SetFlightMode(EAircraftFlightMode::PositionHold);
	AircraftComponent->Arm();
	if (!bPreviewPlaybackEnabled)
	{
		FreezePreviewSimulation();
	}
}

void AAircraftDataflowPreviewActor::RefreshPreviewAssetState()
{
	if (!AircraftComponent)
	{
		return;
	}
	AircraftComponent->RefreshAssetState();
	if (bPreviewFrozen
		&& AircraftComponent->GetCurrentSimulationDriveMode()
			!= EAircraftSimulationDriveMode::Kinematic)
	{
		AircraftComponent->SetAircraftPhysicsSimulationEnabled(false);
	}
}

void AAircraftDataflowPreviewActor::ApplyPreviewHoldTarget(
	const FVector& PositionCm, const float FixedYawDegrees)
{
	PreviewHoldTargetCm = PositionCm;
	PreviewFixedYawDegrees = FixedYawDegrees;
	PreviewMovementIntent.Type = EAircraftMovementIntentType::Hold;
	PreviewMovementIntent.Hold.PositionCm = PositionCm;
	PreviewMovementIntent.Hold.bCaptureCurrentPosition = false;
	PreviewMovementIntent.bHasRequestedMotionLimits = false;
	PreviewMovementIntent.Heading.Mode = EAircraftHeadingMode::FixedYaw;
	PreviewMovementIntent.Heading.FixedYawDegrees = FixedYawDegrees;
	PreviewMovementIntent.TimeoutSeconds = 0.0f;
	++PreviewMovementIntentRevision;
	bPreviewIntentActive = true;
}

void AAircraftDataflowPreviewActor::ApplyPreviewSimulationLOD(const int32 LodIndex)
{
	if (!AircraftComponent)
	{
		return;
	}
	FAircraftSimulationBudget Budget;
	Budget.LODIndex = LodIndex;
	IAircraftSimulationLODConsumer::Execute_ApplyAircraftSimulationBudget(
		AircraftComponent, Budget);
	if (bPreviewFrozen
		&& AircraftComponent->GetCurrentSimulationDriveMode()
			!= EAircraftSimulationDriveMode::Kinematic)
	{
		AircraftComponent->SetAircraftPhysicsSimulationEnabled(false);
	}
}

void AAircraftDataflowPreviewActor::SetPreviewArmed(const bool bArmed)
{
	if (AircraftComponent)
	{
		bArmed ? AircraftComponent->Arm() : AircraftComponent->Disarm();
	}
}

void AAircraftDataflowPreviewActor::SetPreviewFlightMode(const EAircraftFlightMode FlightMode)
{
	if (AircraftComponent)
	{
		AircraftComponent->SetFlightMode(FlightMode);
	}
}

bool AAircraftDataflowPreviewActor::SetPreviewRotorEffectiveness(
	const FName RotorName, const float Effectiveness)
{
	return AircraftComponent
		&& AircraftComponent->SetRotorEffectiveness(RotorName, Effectiveness);
}

void AAircraftDataflowPreviewActor::ApplyPreviewForceAndTorque(
	const FVector& ForceWorldN, const FVector& TorqueWorldNm)
{
	if (!AircraftComponent || bPreviewFrozen)
	{
		return;
	}
	const FName BoneName = GetChassisBoneName();
	AircraftComponent->AddForce(
		AircraftPhysicsUnits::NewtonsToChaosForce(ForceWorldN), BoneName, false);
	AircraftComponent->AddTorqueInRadians(
		AircraftPhysicsUnits::NewtonMetersToChaosTorque(TorqueWorldNm), BoneName, false);
}

void AAircraftDataflowPreviewActor::OnDataflowSimulationEnabledChanged_Implementation(
	const bool bSimulationEnabled)
{
	bPreviewPlaybackEnabled = bSimulationEnabled;
	if (bSimulationEnabled)
	{
		ResumePreviewSimulation();
	}
	else
	{
		FreezePreviewSimulation();
	}
}

bool AAircraftDataflowPreviewActor::GetAircraftMovementIntent(
	FAircraftMovementIntent& OutIntent, FAircraftMovementIntentHandle& OutHandle,
	uint64& OutRevision) const
{
	if (!bPreviewIntentActive)
	{
		return false;
	}
	OutIntent = PreviewMovementIntent;
	OutHandle = PreviewMovementIntentHandle;
	OutRevision = PreviewMovementIntentRevision;
	return true;
}

bool AAircraftDataflowPreviewActor::IsAircraftMovementIntentActive() const
{
	return bPreviewIntentActive;
}

void AAircraftDataflowPreviewActor::OnAircraftMovementIntentInterrupted(
	const FAircraftMovementIntentHandle Handle,
	const EAircraftMovementFailureReason Reason)
{
	(void)Reason;
	if (Handle == PreviewMovementIntentHandle)
	{
		bPreviewIntentActive = false;
		++PreviewMovementIntentRevision;
	}
}

FName AAircraftDataflowPreviewActor::GetChassisBoneName() const
{
	if (AircraftComponent)
	{
		if (const FAircraftSimulationLodModel* const Model = AircraftComponent->GetCurrentLodModel())
		{
			return Model->RootBone;
		}
	}
	return NAME_None;
}

void AAircraftDataflowPreviewActor::FreezePreviewSimulation()
{
	if (bPreviewFrozen || !AircraftComponent)
	{
		return;
	}
	bPreviewFrozen = true;
	AircraftComponent->CapturePhysicsStateSnapshot(FrozenPhysicsState);
	AircraftComponent->SuspendSimulation();
	AircraftComponent->DestroySimulationConstraint();
	AircraftComponent->SetComponentTickEnabled(false);
	if (AircraftComponent->GetCurrentSimulationDriveMode() != EAircraftSimulationDriveMode::Kinematic)
	{
		AircraftComponent->SetAircraftPhysicsSimulationEnabled(false);
	}
}

void AAircraftDataflowPreviewActor::ResumePreviewSimulation()
{
	if (!AircraftComponent)
	{
		return;
	}
	if (!bPreviewFrozen)
	{
		// The native Dataflow manager may already be playing when a fresh preview
		// actor is spawned. In that case there is no frozen state to restore, but
		// the component still has to be made an active simulation participant.
		AircraftComponent->ResumeSimulation();
		AircraftComponent->SetComponentTickEnabled(true);
		return;
	}
	const EAircraftSimulationDriveMode DriveMode = AircraftComponent->GetCurrentSimulationDriveMode();
	if (DriveMode != EAircraftSimulationDriveMode::Kinematic)
	{
		AircraftComponent->SetAircraftPhysicsSimulationEnabled(true);
	}
	AircraftComponent->RestorePhysicsStateSnapshot(FrozenPhysicsState, true);
	AircraftComponent->ResumeSimulation();
	AircraftComponent->SetComponentTickEnabled(true);
	bPreviewFrozen = false;
}

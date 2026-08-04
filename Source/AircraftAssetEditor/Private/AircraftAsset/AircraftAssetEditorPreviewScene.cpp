#include "AircraftAsset/AircraftAssetEditorPreviewScene.h"

#include "Engine/CollisionProfile.h"
#include "AircraftAsset/AircraftAssetBase.h"
#include "AircraftAsset/AircraftComponent.h"
#include "GameFramework/Actor.h"
#include "SkinnedAssetCompiler.h"

FAircraftAssetEditorPreviewScene::FAircraftAssetEditorPreviewScene(FPreviewScene::ConstructionValues ConstructionValues)
	: FAdvancedPreviewScene(ConstructionValues)
{
	SceneActor = GetWorld()->SpawnActor<AActor>(AActor::StaticClass());
	AircraftComponent = NewObject<UAircraftComponent>(SceneActor);
	SceneActor->SetRootComponent(AircraftComponent);
	AircraftComponent->RegisterComponentWithWorld(GetWorld());

	SetFloorVisibility(true, true);
}

FAircraftAssetEditorPreviewScene::~FAircraftAssetEditorPreviewScene()
{
	if (AircraftComponent)
	{
		AircraftComponent->UnregisterComponent();
	}

	if (SceneActor)
	{
		SceneActor->Destroy();
	}
}

void FAircraftAssetEditorPreviewScene::AddReferencedObjects(FReferenceCollector& Collector)
{
	FAdvancedPreviewScene::AddReferencedObjects(Collector);
	Collector.AddReferencedObject(SceneActor);
	Collector.AddReferencedObject(AircraftComponent);
}

void FAircraftAssetEditorPreviewScene::SetAircraftAsset(UAircraftAssetBase* InAircraftAsset)
{
	check(SceneActor);
	check(AircraftComponent);

	UAircraftComponent* const PreviewAircraftComponent = GetAircraftComponent();
	if (!PreviewAircraftComponent)
	{
		return;
	}

	if (InAircraftAsset)
	{
		USkinnedAsset* AssetsToCompile[] = { InAircraftAsset };
		FSkinnedAssetCompilingManager::Get().FinishCompilation(TArrayView<USkinnedAsset* const>(AssetsToCompile, UE_ARRAY_COUNT(AssetsToCompile)));
	}

	USkeletalMesh* SkeletalMesh = nullptr;
#if WITH_EDITORONLY_DATA
	if (InAircraftAsset)
	{
		SkeletalMesh = InAircraftAsset->GetPreviewSceneSkeletalMesh();
	}
	#endif
	AircraftComponent->SetSkeletalMeshAsset(SkeletalMesh);

	PreviewAircraftComponent->SetAsset(InAircraftAsset);
	InitializePreviewAircraft();
}

void FAircraftAssetEditorPreviewScene::SoftResetSimulation()
{
	if (UAircraftComponent* const PreviewAircraftComponent = GetAircraftComponent())
	{
		PreviewAircraftComponent->SoftResetSimulation();
	}
}

void FAircraftAssetEditorPreviewScene::HardResetSimulation()
{
	UAircraftComponent* const PreviewAircraftComponent = GetAircraftComponent();
	if (!PreviewAircraftComponent)
	{
		return;
	}

	const bool bWasSimulationEnabled = PreviewAircraftComponent->IsSimulationEnabled();
	const bool bWasSimulationSuspended = PreviewAircraftComponent->IsSimulationSuspended();

	PreviewAircraftComponent->SetEnableSimulation(false);
	PreviewAircraftComponent->SetAllPhysicsLinearVelocity(FVector::ZeroVector, false);
	PreviewAircraftComponent->SetAllPhysicsAngularVelocityInRadians(FVector::ZeroVector, false);
	PreviewAircraftComponent->SetAllPhysicsPosition(InitialAircraftTransform.GetLocation());
	PreviewAircraftComponent->SetAllPhysicsRotation(InitialAircraftTransform.GetRotation());
	PreviewAircraftComponent->PutAllRigidBodiesToSleep();
	PreviewAircraftComponent->HardResetSimulation();

	if (bWasSimulationEnabled)
	{
		PreviewAircraftComponent->SetEnableSimulation(true);
		if (bWasSimulationSuspended)
		{
			PreviewAircraftComponent->SuspendSimulation();
		}
		else
		{
			PreviewAircraftComponent->ResumeSimulation();
		}
	}
}

void FAircraftAssetEditorPreviewScene::SuspendSimulation()
{
	if (UAircraftComponent* const PreviewAircraftComponent = GetAircraftComponent())
	{
		PreviewAircraftComponent->SuspendSimulation();
	}
}

void FAircraftAssetEditorPreviewScene::ResumeSimulation()
{
	if (UAircraftComponent* const PreviewAircraftComponent = GetAircraftComponent())
	{
		PreviewAircraftComponent->ResumeSimulation();
	}
}

bool FAircraftAssetEditorPreviewScene::IsSimulationSuspended() const
{
	if (const UAircraftComponent* const PreviewAircraftComponent = GetAircraftComponent())
	{
		return PreviewAircraftComponent->IsSimulationSuspended();
	}

	return false;
}

void FAircraftAssetEditorPreviewScene::SetEnableSimulation(bool bEnable)
{
	if (UAircraftComponent* const PreviewAircraftComponent = GetAircraftComponent())
	{
		PreviewAircraftComponent->SetEnableSimulation(bEnable);
	}
}

bool FAircraftAssetEditorPreviewScene::IsSimulationEnabled() const
{
	if (const UAircraftComponent* const PreviewAircraftComponent = GetAircraftComponent())
	{
		return PreviewAircraftComponent->IsSimulationEnabled();
	}

	return false;
}

UAircraftComponent* FAircraftAssetEditorPreviewScene::GetAircraftComponent() const
{
	return AircraftComponent;
}

void FAircraftAssetEditorPreviewScene::InitializePreviewAircraft()
{
	UAircraftComponent* const PreviewAircraftComponent = GetAircraftComponent();
	if (!PreviewAircraftComponent)
	{
		return;
	}

	if (!PreviewAircraftComponent->GetAsset() || !PreviewAircraftComponent->GetSkeletalMeshAsset() || !PreviewAircraftComponent->GetPhysicsAsset())
	{
		PreviewAircraftComponent->SetEnableSimulation(false);
		PreviewAircraftComponent->SetSimulatePhysics(false);
		PreviewAircraftComponent->SetCollisionEnabled(ECollisionEnabled::NoCollision);
		InitialAircraftTransform = FTransform::Identity;
		PreviewAircraftComponent->SetWorldTransform(InitialAircraftTransform, false, nullptr, ETeleportType::ResetPhysics);
		return;
	}

	PreviewAircraftComponent->SetCollisionProfileName(UCollisionProfile::PhysicsActor_ProfileName);
	PreviewAircraftComponent->SetCollisionEnabled(ECollisionEnabled::QueryAndPhysics);
	PreviewAircraftComponent->SetEnableSimulation(true);
	PreviewAircraftComponent->SetSimulatePhysics(false);

	UpdateInitialAircraftTransform();
	PreviewAircraftComponent->SetWorldTransform(InitialAircraftTransform, false, nullptr, ETeleportType::ResetPhysics);
	PreviewAircraftComponent->SetSimulatePhysics(true);
	PreviewAircraftComponent->SetAllPhysicsLinearVelocity(FVector::ZeroVector, false);
	PreviewAircraftComponent->SetAllPhysicsAngularVelocityInRadians(FVector::ZeroVector, false);
	PreviewAircraftComponent->SetAllPhysicsPosition(InitialAircraftTransform.GetLocation());
	PreviewAircraftComponent->SetAllPhysicsRotation(InitialAircraftTransform.GetRotation());
	PreviewAircraftComponent->WakeAllRigidBodies();
	PreviewAircraftComponent->HardResetSimulation();
}

void FAircraftAssetEditorPreviewScene::UpdateInitialAircraftTransform()
{
	InitialAircraftTransform = FTransform::Identity;

	if (const UAircraftComponent* const PreviewAircraftComponent = GetAircraftComponent())
	{
		const FBoxSphereBounds AircraftBounds = PreviewAircraftComponent->CalcBounds(FTransform::Identity);
		const float InitialHeight = FMath::Max(50.f, AircraftBounds.BoxExtent.Z + 10.f);
		InitialAircraftTransform = FTransform(FQuat::Identity, FVector(0.f, 0.f, InitialHeight));
	}
}

#include "AircraftAsset/AircraftAssetEditorPreviewScene.h"

#include "Editor.h"
#include "Engine/CollisionProfile.h"
#include "AircraftAsset/AircraftAssetBase.h"
#include "AircraftAsset/AircraftComponent.h"
#include "AircraftAsset/AircraftPreviewSceneDescription.h"
#include "Animation/AnimSingleNodeInstance.h"
#include "Components/SkeletalMeshComponent.h"
#include "GameFramework/Actor.h"
#include "SkinnedAssetCompiler.h"
#include "UObject/PackageReload.h"
#include "EditorReimportHandler.h"

FAircraftAssetEditorPreviewScene::FAircraftAssetEditorPreviewScene(FPreviewScene::ConstructionValues ConstructionValues)
	: FAdvancedPreviewScene(ConstructionValues)
{
	PreviewSceneDescription = NewObject<UAircraftPreviewSceneDescription>();
	PreviewSceneDescription->SetPreviewScene(this);

	SceneActor = GetWorld()->SpawnActor<AActor>(AActor::StaticClass());
	AircraftComponent = NewObject<UAircraftComponent>(SceneActor);
	SceneActor->SetRootComponent(AircraftComponent);
	AircraftComponent->RegisterComponentWithWorld(GetWorld());

	SetFloorVisibility(true, true);

	OnPackageReloadedDelegateHandle = FCoreUObjectDelegates::OnPackageReloaded.AddRaw(this, &FAircraftAssetEditorPreviewScene::HandlePackageReloaded);
	OnPostReimportDelegateHandle = FReimportManager::Instance()->OnPostReimport().AddRaw(this, &FAircraftAssetEditorPreviewScene::HandleReimportManagerPostReimport);
}

FAircraftAssetEditorPreviewScene::~FAircraftAssetEditorPreviewScene()
{
	FCoreUObjectDelegates::OnPackageReloaded.Remove(OnPackageReloadedDelegateHandle);
	FReimportManager::Instance()->OnPostReimport().Remove(OnPostReimportDelegateHandle);

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
	Collector.AddReferencedObject(PreviewSceneDescription);
}

void FAircraftAssetEditorPreviewScene::Tick(float DeltaT)
{
	// 预览网格重载/重导入后，动画实例会在组件重注册延迟完成后重建；
	// 这里延迟到 Tick 中恢复之前保存的播放状态（对齐 FChaosClothPreviewScene::Tick）。
	FAdvancedPreviewScene::Tick(DeltaT);

	if (SavedAnimState)
	{
		RestoreSavedAnimationState();
	}

	// PIE/SIE 期间按描述对象设置暂停预览开场动画（仿真自身由组件开关控制）。
	if (PreviewSceneDescription && PreviewSceneDescription->bPauseWhilePlayingInEditor)
	{
		if (UAnimSingleNodeInstance* const AnimInstance = GetPreviewAnimInstance())
		{
			if (GEditor && (GEditor->PlayWorld || GEditor->bIsSimulatingInEditor))
			{
				AnimInstance->SetPlaying(false);
			}
		}
	}
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

	if (PreviewSceneDescription && SkeletalMesh != PreviewSceneDescription->SkeletalMeshAsset)
	{
		SaveAnimationState();
		PreviewSceneDescription->SkeletalMeshAsset = SkeletalMesh;
		PreviewAircraftComponent->SetSkeletalMeshAsset(SkeletalMesh);
		UpdateSkeletalMeshAnimation();
	}
	else
	{
		PreviewAircraftComponent->SetSkeletalMeshAsset(SkeletalMesh);
	}

#if WITH_EDITORONLY_DATA
	if (InAircraftAsset && PreviewSceneDescription)
	{
		if (UAnimationAsset* const Animation = InAircraftAsset->GetPreviewSceneAnimation())
		{
			if (PreviewSceneDescription->AnimationAsset != Animation)
			{
				PreviewSceneDescription->AnimationAsset = Animation;
				UpdateSkeletalMeshAnimation();
			}
		}
	}
#endif

	PreviewAircraftComponent->SetAsset(InAircraftAsset);
	InitializePreviewAircraft();
}

UAnimSingleNodeInstance* FAircraftAssetEditorPreviewScene::GetPreviewAnimInstance()
{
	check(AircraftComponent);
	if (AircraftComponent->AnimScriptInstance)
	{
		return CastChecked<UAnimSingleNodeInstance>(AircraftComponent->AnimScriptInstance);
	}
	return nullptr;
}

const UAnimSingleNodeInstance* FAircraftAssetEditorPreviewScene::GetPreviewAnimInstance() const
{
	check(AircraftComponent);
	if (AircraftComponent->AnimScriptInstance)
	{
		return CastChecked<UAnimSingleNodeInstance>(AircraftComponent->AnimScriptInstance);
	}
	return nullptr;
}

void FAircraftAssetEditorPreviewScene::SceneDescriptionPropertyChanged(const FName& PropertyName)
{
	if (!PreviewSceneDescription)
	{
		return;
	}

	UAircraftComponent* const PreviewAircraftComponent = GetAircraftComponent();
	if (!PreviewAircraftComponent)
	{
		return;
	}

	if (PropertyName == GET_MEMBER_NAME_CHECKED(UAircraftPreviewSceneDescription, SkeletalMeshAsset))
	{
		SaveAnimationState();
		PreviewAircraftComponent->SetSkeletalMeshAsset(PreviewSceneDescription->SkeletalMeshAsset);
		UpdateSkeletalMeshAnimation();

		if (UAircraftAssetBase* const AircraftAsset = Cast<UAircraftAssetBase>(PreviewAircraftComponent->GetAsset()))
		{
			AircraftAsset->SetPreviewSceneSkeletalMesh(PreviewSceneDescription->SkeletalMeshAsset);
		}

		InitializePreviewAircraft();
	}

	if (PropertyName == GET_MEMBER_NAME_CHECKED(UAircraftPreviewSceneDescription, AnimationAsset))
	{
		UpdateSkeletalMeshAnimation();

		if (UAircraftAssetBase* const AircraftAsset = Cast<UAircraftAssetBase>(PreviewAircraftComponent->GetAsset()))
		{
			AircraftAsset->SetPreviewSceneAnimation(PreviewSceneDescription->AnimationAsset);
		}
	}
}

void FAircraftAssetEditorPreviewScene::UpdateSkeletalMeshAnimation()
{
	check(AircraftComponent);
	check(PreviewSceneDescription);

	// 与布料不同：无人机的"机体网格"即 AircraftComponent 自身，
	// 不需要额外的跟随式骨架组件，动画直接播放在 AircraftComponent 上。
	const bool bWasPlaying = AircraftComponent->IsPlaying();
	AircraftComponent->Stop();

	if (PreviewSceneDescription->AnimationAsset)
	{
		TObjectPtr<UAnimSingleNodeInstance> PreviewAnimInstance = NewObject<UAnimSingleNodeInstance>(AircraftComponent);
		PreviewAnimInstance->SetAnimationAsset(PreviewSceneDescription->AnimationAsset);

		AircraftComponent->SetAnimationMode(EAnimationMode::AnimationSingleNode);
		AircraftComponent->InitAnim(true);
		AircraftComponent->AnimationData.PopulateFrom(PreviewAnimInstance);
		AircraftComponent->AnimScriptInstance = PreviewAnimInstance;
		AircraftComponent->AnimScriptInstance->InitializeAnimation();
		AircraftComponent->ValidateAnimation();

		if (!bWasPlaying)
		{
			AircraftComponent->Stop();
		}
	}
	else
	{
		AircraftComponent->AnimationData = FSingleAnimationPlayData();
		AircraftComponent->AnimScriptInstance = nullptr;
		// 无动画时恢复默认（无动画蓝图）模式
		AircraftComponent->SetAnimationMode(EAnimationMode::AnimationBlueprint);
	}
}

void FAircraftAssetEditorPreviewScene::SaveAnimationState()
{
	if (const UAnimSingleNodeInstance* const AnimInstance = GetPreviewAnimInstance())
	{
		SavedAnimState = FAnimState();
		SavedAnimState->Time = AnimInstance->GetCurrentTime();
		SavedAnimState->bIsReverse = AnimInstance->IsReverse();
		SavedAnimState->bIsLooping = AnimInstance->IsLooping();
		SavedAnimState->bIsPlaying = AnimInstance->IsPlaying();
	}
}

void FAircraftAssetEditorPreviewScene::RestoreSavedAnimationState()
{
	if (SavedAnimState)
	{
		if (UAnimSingleNodeInstance* const AnimInstance = GetPreviewAnimInstance())
		{
			AnimInstance->SetPosition(SavedAnimState->Time);
			AnimInstance->SetReverse(SavedAnimState->bIsReverse);
			AnimInstance->SetLooping(SavedAnimState->bIsLooping);
			AnimInstance->SetPlaying(SavedAnimState->bIsPlaying);
		}
		SavedAnimState.Reset();
	}
}

void FAircraftAssetEditorPreviewScene::HandlePackageReloaded(const EPackageReloadPhase InPackageReloadPhase, FPackageReloadedEvent* InPackageReloadedEvent)
{
	if (InPackageReloadPhase == EPackageReloadPhase::PrePackageFixup && PreviewSceneDescription)
	{
		for (const TPair<UObject*, UObject*>& RepointPair : InPackageReloadedEvent->GetRepointedObjects())
		{
			if (RepointPair.Key == PreviewSceneDescription->SkeletalMeshAsset.Get())
			{
				// 网格即将重载：先保存动画状态（组件重注册会重建 AnimInstance），
				// 在 Tick 中恢复（重载委托之后 AnimInstance 才被重建）。
				SaveAnimationState();
			}
		}
	}
}

void FAircraftAssetEditorPreviewScene::HandleReimportManagerPostReimport(UObject* ReimportedObject, bool bWasSuccessful)
{
	if (PreviewSceneDescription && ReimportedObject == PreviewSceneDescription->SkeletalMeshAsset && bWasSuccessful)
	{
		SaveAnimationState();
	}
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

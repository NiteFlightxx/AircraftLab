// Fill out your copyright notice in the Description page of Project Settings.


#include "AircraftAsset/AircraftAssetThumbnailRenderer.h"

#include "Engine/SkeletalMesh.h"
#include "SceneView.h"
#include "ShowFlags.h"
#include "ThumbnailRendering/SceneThumbnailInfo.h"
#include "AircraftAsset/AircraftAsset.h"
#include "AircraftAsset/AircraftComponent.h"

#include UE_INLINE_GENERATED_CPP_BY_NAME(AircraftAssetThumbnailRenderer)

AAircraftPreviewActor::AAircraftPreviewActor()
{
	AircraftComponent = CreateDefaultSubobject<UAircraftComponent>(TEXT("AircraftComponent0"));
	RootComponent = AircraftComponent;
}

namespace UE::AircraftLab::AircraftAsset
{
	FThumbnailScene::FThumbnailScene()
		: FThumbnailPreviewScene()
	{
		bForceAllUsedMipsResident = false;

		FActorSpawnParameters SpawnInfo;
		SpawnInfo.SpawnCollisionHandlingOverride = ESpawnActorCollisionHandlingMethod::AlwaysSpawn;
		SpawnInfo.bNoFail = true;
		SpawnInfo.ObjectFlags = RF_Transient;

		PreviewActor = GetWorld()->SpawnActor<AAircraftPreviewActor>(SpawnInfo);
		PreviewActor->SetActorEnableCollision(false);

		check(PreviewActor);
	}

	void FThumbnailScene::SetAircraftAsset(UAircraftAssetBase* InAircraftAsset)
	{
		UAircraftComponent* AircraftComponent = PreviewActor->GetAircraftComponent();
		check(AircraftComponent);
		
		AircraftComponent->SetAsset(InAircraftAsset);
		
	}

	void FThumbnailScene::GetViewMatrixParameters(const float InFOVDegrees, FVector& OutOrigin, float& OutOrbitPitch,
		float& OutOrbitYaw, float& OutOrbitZoom) const
	{
		UAircraftComponent* AircraftComponent = PreviewActor->GetAircraftComponent();
		check(AircraftComponent);

		FBoxSphereBounds Bounds = AircraftComponent->Bounds;
		Bounds = Bounds.ExpandBy(2.0f);
		
		const float HalfFOVRadians = FMath::DegreesToRadians<float>(InFOVDegrees) * 0.5f;
		const float HalfMeshSize = static_cast<float>(Bounds.SphereRadius); 
		const float TargetDistance = HalfMeshSize / FMath::Tan(HalfFOVRadians);

		USceneThumbnailInfo* ThumbnailInfo = Cast<USceneThumbnailInfo>(AircraftComponent->GetThumbnailInfo());
		if (ThumbnailInfo)
		{
			if (TargetDistance + ThumbnailInfo->OrbitZoom < 0 )
			{
				ThumbnailInfo->OrbitZoom = -TargetDistance;
			}
		}
		else
		{
			ThumbnailInfo = USceneThumbnailInfo::StaticClass()->GetDefaultObject<USceneThumbnailInfo>();
		}

		OutOrigin = -Bounds.Origin;
		OutOrbitPitch = ThumbnailInfo->OrbitPitch;
		OutOrbitYaw = ThumbnailInfo->OrbitYaw;
		OutOrbitZoom = TargetDistance + ThumbnailInfo->OrbitZoom;
	}
}

void UAircraftAssetThumbnailRenderer::Draw(UObject* Object, int32 X, int32 Y, uint32 Width, uint32 Height,
	FRenderTarget* Viewport, FCanvas* Canvas, bool bAdditionalViewFamily)
{
	using namespace UE::AircraftLab::AircraftAsset;
	
	UAircraftAssetBase* const AircraftAsset = Cast<UAircraftAssetBase>(Object);
	if (!AircraftAsset)
	{
		return;
	}
	
	TSharedRef<FThumbnailScene> ThumbnailScene = AircraftThumbnailSceneCache.EnsureThumbnailScene(Object);
	
	ThumbnailScene->SetAircraftAsset(AircraftAsset);
	
	FSceneViewFamilyContext ViewFamily(FSceneViewFamily::ConstructionValues(Viewport, ThumbnailScene->GetScene(), FEngineShowFlags(ESFIM_Game))
		.SetTime(UThumbnailRenderer::GetTime())
		.SetAdditionalViewFamily(bAdditionalViewFamily));

	ViewFamily.EngineShowFlags.DisableAdvancedFeatures();
	ViewFamily.EngineShowFlags.MotionBlur = 0;
	ViewFamily.EngineShowFlags.LOD = 0;

	RenderViewFamily(Canvas, &ViewFamily, ThumbnailScene->CreateView(&ViewFamily, X, Y, Width, Height));
	ThumbnailScene->SetAircraftAsset(nullptr);
	
}

bool UAircraftAssetThumbnailRenderer::CanVisualizeAsset(UObject* Object)
{
	bool bCanVisualize= Cast<UAircraftAssetBase>(Object) != nullptr;
	return bCanVisualize;
}

EThumbnailRenderFrequency UAircraftAssetThumbnailRenderer::GetThumbnailRenderFrequency(UObject* Object) const
{
	UAircraftAssetBase* AsAircraftAsset = Cast<UAircraftAssetBase>(Object);
	return AsAircraftAsset && AsAircraftAsset->GetResourceForRendering() ? EThumbnailRenderFrequency::Realtime : EThumbnailRenderFrequency::OnPropertyChange;
}

void UAircraftAssetThumbnailRenderer::BeginDestroy()
{
	AircraftThumbnailSceneCache.Clear();
	Super::BeginDestroy();
}

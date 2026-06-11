// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "ThumbnailHelpers.h"
#include "ThumbnailRendering/DefaultSizedThumbnailRenderer.h"
#include "AircraftAsset/AircraftAssetBase.h"
#include "AircraftAssetThumbnailRenderer.generated.h"

class UAircraftComponent;

UCLASS(MinimalAPI)
class AAircraftPreviewActor : public AActor
{
	GENERATED_BODY()

public:
	AAircraftPreviewActor();

	UAircraftComponent* GetAircraftComponent()
	{
		return AircraftComponent;
	}

protected:
	UPROPERTY()
	TObjectPtr<UAircraftComponent> AircraftComponent;
};

/**
 * 
 */

namespace UE::AircraftLab::AircraftAsset
{
	/**
	 * Preview scene for a Aircraft asset thumbnail
	 */
	class FThumbnailScene : public FThumbnailPreviewScene
	{
	public:
		FThumbnailScene();

		void SetAircraftAsset(UAircraftAssetBase* InAircraftAsset);

		virtual void GetViewMatrixParameters(const float InFOVDegrees, FVector& OutOrigin, float& OutOrbitPitch, float& OutOrbitYaw, float& OutOrbitZoom) const override;

	private:
		AAircraftPreviewActor* PreviewActor = nullptr;
	};
}

UCLASS()
class AIRCRAFTASSETEDITOR_API UAircraftAssetThumbnailRenderer : public UDefaultSizedThumbnailRenderer
{
	GENERATED_BODY()

	virtual void Draw(UObject* Object, int32 X, int32 Y, uint32 Width, uint32 Height, FRenderTarget* Viewport, FCanvas* Canvas, bool bAdditionalViewFamily) override;
	virtual bool CanVisualizeAsset(UObject* Object) override;
	virtual EThumbnailRenderFrequency GetThumbnailRenderFrequency(UObject* Object) const override;

	virtual void BeginDestroy() override;

protected:
	TObjectInstanceThumbnailScene<UE::AircraftLab::AircraftAsset::FThumbnailScene, 128> AircraftThumbnailSceneCache;
};

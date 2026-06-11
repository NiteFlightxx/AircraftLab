// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "AircraftAssetBase.h"
#include "AircraftAsset.generated.h"

UCLASS(hidecategories = Object, BlueprintType, PrioritizeCategories = ("Dataflow"))
class AIRCRAFTASSETENGINE_API UAircraftAsset : public UAircraftAssetBase
{
	GENERATED_BODY()

public:
	UAircraftAsset(const FObjectInitializer& ObjectInitializer);
	UAircraftAsset(FVTableHelper& Helper);
	virtual ~UAircraftAsset() override;

	//~ Begin UObject interface
	virtual void Serialize(FArchive& Ar) override;
#if WITH_EDITOR
	virtual void PostEditChangeProperty(FPropertyChangedEvent& PropertyChangedEvent) override;
#endif
	//~ End UObject interface

	//~ Begin USkinnedAsset interface
	virtual UPhysicsAsset* GetPhysicsAsset() const override
	{
		return PhysicsAsset;
	}

	virtual USkeleton* GetSkeleton() override
	{
		return Skeleton;
	}

	virtual const USkeleton* GetSkeleton() const override
	{
		return Skeleton;
	}

	virtual void SetSkeleton(USkeleton* InSkeleton) override
	{
		Skeleton = InSkeleton;
	}

#if WITH_EDITOR
	virtual FString BuildDerivedDataKey(const ITargetPlatform* TargetPlatform) override;
	virtual bool IsInitialBuildDone() const override;
#endif

#if WITH_EDITORONLY_DATA
	virtual FSkeletalMeshModel* GetImportedModel() const override;
#endif
	//~ End USkinnedAsset interface

	void Build(
		const TArray<TSharedRef<const FManagedArrayCollection>>& InAircraftCollections,
		FText* ErrorText = nullptr,
		FText* VerboseText = nullptr);

	//~ Begin UAircraftAssetBase interface
	virtual bool HasValidAircraftSimulationModels() const override;
	virtual int32 GetNumAircraftSimulationModels() const override
	{
		return AircraftSimulationModel.IsValid() ? 1 : 0;
	}

	virtual TSharedPtr<const FAircraftSimulationModel> GetAircraftSimulationModel(int32 /*ModelIndex*/) const override
	{
		return AircraftSimulationModel;
	}

	virtual void SetCollections(TArray<TSharedRef<const FManagedArrayCollection>>&& InCollections) override;
	virtual const TArray<TSharedRef<const FManagedArrayCollection>>& GetCollections(int32 /*ModelIndex*/) const override;
	//~ End UAircraftAssetBase interface

	const TArray<TSharedRef<const FManagedArrayCollection>>& GetAircraftCollections() const;
	void SetAircraftCollections(TArray<TSharedRef<const FManagedArrayCollection>>&& InAircraftCollections);

protected:
	virtual USkeletalMesh* GetSourceSkeletalMesh() const override;

private:
	virtual void BeginPostLoadInternal(FSkinnedAssetPostLoadContext& Context) override;

	TArray<TSharedRef<const FManagedArrayCollection>>& GetAircraftCollectionsInternal();
	void EnsureCollectionsInitialized();
	void SynchronizeAssetStateFromCollections();
	void BuildAircraftSimulationModel();

private:
	UPROPERTY(EditAnywhere, Setter = SetSkeleton, Category = Skeleton)
	TObjectPtr<USkeleton> Skeleton;

	UPROPERTY(EditAnywhere, Category = Collision)
	TObjectPtr<UPhysicsAsset> PhysicsAsset;

	TArray<TSharedRef<const FManagedArrayCollection>> AircraftCollections;
	TSharedPtr<FAircraftSimulationModel> AircraftSimulationModel;
};

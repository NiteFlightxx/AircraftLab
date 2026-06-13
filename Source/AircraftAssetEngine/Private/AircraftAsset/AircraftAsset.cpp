#include "AircraftAsset/AircraftAsset.h"

#include "Engine/SkeletalMesh.h"
#include "PhysicsEngine/PhysicsAsset.h"
#include "UObject/SoftObjectPath.h"
#include "AircraftAsset/AircraftCollection.h"
#include "AircraftAsset/AircraftSimulationModel.h"

#define LOCTEXT_NAMESPACE "AircraftAsset"

namespace
{
	FArchive& SerializeCollections(FArchive& Ar, TArray<TSharedRef<const FManagedArrayCollection>>& InOutAircraftCollections)
	{
		InOutAircraftCollections.CountBytes(Ar);

		int32 SerializeNum = Ar.IsLoading() ? 0 : InOutAircraftCollections.Num();
		Ar << SerializeNum;

		if (SerializeNum == 0)
		{
			if (Ar.IsLoading())
			{
				InOutAircraftCollections.Empty();
			}
			return Ar;
		}

		check(SerializeNum >= 0);
		if (Ar.IsError() || SerializeNum < 0)
		{
			Ar.SetError();
			return Ar;
		}

		if (Ar.IsLoading())
		{
			InOutAircraftCollections.Empty(SerializeNum);

			for (int32 CollectionIndex = 0; CollectionIndex < SerializeNum; ++CollectionIndex)
			{
				TSharedRef<FManagedArrayCollection> AircraftCollection = MakeShared<FManagedArrayCollection>();
				AircraftCollection->Serialize(Ar);

				UE::AircraftLab::AircraftAsset::FAircraftCollection AircraftFacade(AircraftCollection);
				AircraftFacade.DefineSchema();

				InOutAircraftCollections.Emplace(MoveTemp(AircraftCollection));
			}
		}
		else
		{
			check(SerializeNum == InOutAircraftCollections.Num());

			for (int32 CollectionIndex = 0; CollectionIndex < SerializeNum; ++CollectionIndex)
			{
				ConstCastSharedRef<FManagedArrayCollection>(InOutAircraftCollections[CollectionIndex])->Serialize(Ar);
			}
		}

		return Ar;
	}

	FSoftObjectPath GetFirstPath(const TManagedArray<FSoftObjectPath>* Paths)
	{
		return Paths && Paths->Num() > 0 ? (*Paths)[0] : FSoftObjectPath();
	}

	USkeletalMesh* ResolveSourceSkeletalMesh(const TArray<TSharedRef<const FManagedArrayCollection>>& InAircraftCollections)
	{
		if (InAircraftCollections.IsEmpty())
		{
			return nullptr;
		}

		const UE::AircraftLab::AircraftAsset::FConstAircraftCollection AircraftCollection(InAircraftCollections[0]);
		if (!AircraftCollection.IsValid())
		{
			return nullptr;
		}

		return Cast<USkeletalMesh>(GetFirstPath(AircraftCollection.GetSkeletalMeshSoftObjectPathName()).TryLoad());
	}

	UPhysicsAsset* ResolveSourcePhysicsAsset(const TArray<TSharedRef<const FManagedArrayCollection>>& InAircraftCollections)
	{
		if (InAircraftCollections.IsEmpty())
		{
			return nullptr;
		}

		const UE::AircraftLab::AircraftAsset::FConstAircraftCollection AircraftCollection(InAircraftCollections[0]);
		if (!AircraftCollection.IsValid())
		{
			return nullptr;
		}

		return Cast<UPhysicsAsset>(GetFirstPath(AircraftCollection.GetPhysicsAssetSoftObjectPathName()).TryLoad());
	}
}

using namespace UE::AircraftLab::AircraftAsset;

UAircraftAsset::UAircraftAsset(const FObjectInitializer& ObjectInitializer)
	: Super(ObjectInitializer)
{
	DataflowInstance.SetDataflowTerminal(TEXT("AircraftAssetTerminal"));
	EnsureCollectionsInitialized();
}

UAircraftAsset::UAircraftAsset(FVTableHelper& Helper)
	: Super(Helper)
{
}

UAircraftAsset::~UAircraftAsset() = default;

#if WITH_EDITOR
FString UAircraftAsset::BuildDerivedDataKey(const ITargetPlatform* TargetPlatform)
{
	(void)TargetPlatform;

	const USkeletalMesh* const SourceSkeletalMesh = GetSourceSkeletalMesh();
	const FString SkeletalMeshKey = SourceSkeletalMesh ? SourceSkeletalMesh->GetPathName() : TEXT("None");
	const FString PhysicsAssetKey = PhysicsAsset ? PhysicsAsset->GetPathName() : TEXT("None");
	return FString::Printf(TEXT("AircraftAsset_NoRenderData_%s_%s"), *SkeletalMeshKey, *PhysicsAssetKey);
}

bool UAircraftAsset::IsInitialBuildDone() const
{
	return true;
}
#endif

void UAircraftAsset::Build(
	const TArray<TSharedRef<const FManagedArrayCollection>>& InAircraftCollections,
	FText* ErrorText,
	FText* VerboseText)
{
	TArray<TSharedRef<const FManagedArrayCollection>>& OutAircraftCollections = GetAircraftCollectionsInternal();
	OutAircraftCollections.Reset(InAircraftCollections.Num());

	for (int32 LodIndex = 0; LodIndex < InAircraftCollections.Num(); ++LodIndex)
	{
		const FConstAircraftCollection InAircraftCollection(InAircraftCollections[LodIndex]);
		if (!InAircraftCollection.IsValid())
		{
			if (ErrorText && ErrorText->IsEmpty())
			{
				*ErrorText = LOCTEXT("BuildErrorText", "Invalid LOD.");
				if (VerboseText)
				{
					*VerboseText = FText::Format(
						LOCTEXT("BuildVerboseTextFirstError", "LOD {0} has no valid data."),
						LodIndex);
				}
			}
			else if (ErrorText && VerboseText)
			{
				*VerboseText = FText::Format(
					LOCTEXT("BuildVerboseTextThereafter", "{0}\nLOD {1} has no valid data."),
					*VerboseText,
					LodIndex);
			}

			TSharedRef<FManagedArrayCollection> EmptyAircraftCollection = MakeShared<FManagedArrayCollection>();
			FAircraftCollection EmptyAircraftFacade(EmptyAircraftCollection);
			EmptyAircraftFacade.DefineSchema();
			OutAircraftCollections.Emplace(MoveTemp(EmptyAircraftCollection));
			continue;
		}

		TSharedRef<FManagedArrayCollection> AircraftCollection = MakeShared<FManagedArrayCollection>(*InAircraftCollections[LodIndex]);
		FAircraftCollection AircraftFacade(AircraftCollection);
		AircraftFacade.DefineSchema();
		OutAircraftCollections.Emplace(MoveTemp(AircraftCollection));
	}

	EnsureCollectionsInitialized();
	SynchronizeAssetStateFromCollections();
	OnAssetChanged();
}

void UAircraftAsset::Serialize(FArchive& Ar)
{
	Super::Serialize(Ar);
	SerializeCollections(Ar, GetAircraftCollectionsInternal());
}

#if WITH_EDITOR
void UAircraftAsset::PostEditChangeProperty(FPropertyChangedEvent& PropertyChangedEvent)
{
	if (PropertyChangedEvent.GetPropertyName() == GET_MEMBER_NAME_CHECKED(UAircraftAsset, PhysicsAsset))
	{
		OnAssetChanged();
	}

	Super::PostEditChangeProperty(PropertyChangedEvent);
}
#endif

bool UAircraftAsset::HasValidAircraftSimulationModels() const
{
	return AircraftSimulationModel.IsValid() && AircraftSimulationModel->SkeletalMesh != nullptr;
}

void UAircraftAsset::SetCollections(TArray<TSharedRef<const FManagedArrayCollection>>&& InCollections)
{
	SetAircraftCollections(MoveTemp(InCollections));
}

const TArray<TSharedRef<const FManagedArrayCollection>>& UAircraftAsset::GetCollections(int32 /*ModelIndex*/) const
{
	return GetAircraftCollections();
}

const TArray<TSharedRef<const FManagedArrayCollection>>& UAircraftAsset::GetAircraftCollections() const
{
	return AircraftCollections;
}

void UAircraftAsset::SetAircraftCollections(TArray<TSharedRef<const FManagedArrayCollection>>&& InAircraftCollections)
{
	AircraftCollections = MoveTemp(InAircraftCollections);
	OnPropertyChanged();
}

USkeletalMesh* UAircraftAsset::GetSourceSkeletalMesh() const
{
	if (AircraftSimulationModel.IsValid() && AircraftSimulationModel->SkeletalMesh != nullptr)
	{
		return AircraftSimulationModel->SkeletalMesh;
	}

#if WITH_EDITORONLY_DATA
	return GetPreviewSceneSkeletalMesh();
#else
	return nullptr;
#endif
}

void UAircraftAsset::BeginPostLoadInternal(FSkinnedAssetPostLoadContext& Context)
{
	(void)Context;

	EnsureCollectionsInitialized();
	SynchronizeAssetStateFromCollections();
}

TArray<TSharedRef<const FManagedArrayCollection>>& UAircraftAsset::GetAircraftCollectionsInternal()
{
	return AircraftCollections;
}

void UAircraftAsset::EnsureCollectionsInitialized()
{
	if (AircraftCollections.IsEmpty())
	{
		TSharedRef<FManagedArrayCollection> AircraftCollection = MakeShared<FManagedArrayCollection>();
		FAircraftCollection AircraftFacade(AircraftCollection);
		AircraftFacade.DefineSchema();
		AircraftCollections.Emplace(MoveTemp(AircraftCollection));
		return;
	}

	for (int32 CollectionIndex = 0; CollectionIndex < AircraftCollections.Num(); ++CollectionIndex)
	{
		const TSharedRef<const FManagedArrayCollection>& AircraftCollection = AircraftCollections[CollectionIndex];
		const FConstAircraftCollection AircraftConstCollection(AircraftCollection);
		if (AircraftConstCollection.IsValid())
		{
			continue;
		}

		TSharedRef<FManagedArrayCollection> NewAircraftCollection = MakeShared<FManagedArrayCollection>(*AircraftCollection);
		FAircraftCollection NewAircraftFacade(NewAircraftCollection);
		NewAircraftFacade.DefineSchema();
		AircraftCollections[CollectionIndex] = MoveTemp(NewAircraftCollection);
	}
}

void UAircraftAsset::SynchronizeAssetStateFromCollections()
{
	USkeletalMesh* const SkeletalMesh = ResolveSourceSkeletalMesh(GetAircraftCollectionsInternal());

	PhysicsAsset = ResolveSourcePhysicsAsset(GetAircraftCollectionsInternal());
	if (!PhysicsAsset && SkeletalMesh)
	{
		PhysicsAsset = SkeletalMesh->GetPhysicsAsset();
	}

	SetSkeleton(SkeletalMesh ? SkeletalMesh->GetSkeleton() : nullptr);

#if WITH_EDITORONLY_DATA
	SetPreviewSceneSkeletalMesh(SkeletalMesh);
#endif

	BuildAircraftSimulationModel();
}

void UAircraftAsset::BuildAircraftSimulationModel()
{
	AircraftSimulationModel = MakeShared<FAircraftSimulationModel>(
		const_cast<const UAircraftAsset*>(this)->GetAircraftCollections(),
		GetFName());

	// 把资产层引用的 SkeletalMesh / PhysicsAsset 同步到运行时只读模型中，供 SimulationProxy 消费。
	if (AircraftSimulationModel.IsValid())
	{
		AircraftSimulationModel->SkeletalMesh = const_cast<USkeletalMesh*>(GetSourceSkeletalMesh());
		AircraftSimulationModel->PhysicsAsset = PhysicsAsset;
	}
}

#if WITH_EDITORONLY_DATA
FSkeletalMeshModel* UAircraftAsset::GetImportedModel() const
{
	if (const USkeletalMesh* const SourceSkeletalMesh = GetSourceSkeletalMesh())
	{
		return SourceSkeletalMesh->GetImportedModel();
	}

	return nullptr;
}
#endif

#undef LOCTEXT_NAMESPACE

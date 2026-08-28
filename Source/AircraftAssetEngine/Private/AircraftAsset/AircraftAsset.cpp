#include "AircraftAsset/AircraftAsset.h"

#include "Engine/SkeletalMesh.h"
#include "Engine/SkeletalMeshSocket.h"
#include "PhysicsEngine/PhysicsAsset.h"
#include "Misc/SecureHash.h"
#include "Serialization/MemoryWriter.h"
#include "UObject/SoftObjectPath.h"
#include "AircraftAsset/AircraftCollection.h"
#include "AircraftAsset/AircraftAssetCustomVersion.h"
#include "AircraftAsset/CollectionAircraftPropertyFacade.h"
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

	void ResolveRotorSocketTransforms(FAircraftSimulationLodModel& Model, const USkeletalMesh* SkeletalMesh)
	{
		if (!SkeletalMesh)
		{
			return;
		}

		// GetComposedRefPoseMatrix 统一处理骨骼名与 socket 名：先按骨骼查，
		// 查不到则按 socket 查（socket 偏移 × 所挂骨骼的组件空间变换）。
		// 根骨骼的组件空间变换用于把结果转换到机体（根骨骼）坐标系。
		const FReferenceSkeleton& ReferenceSkeleton = SkeletalMesh->GetRefSkeleton();
		const FName RootBoneName = Model.RootBone.IsNone()
			? (ReferenceSkeleton.GetNum() > 0 ? ReferenceSkeleton.GetBoneName(0) : NAME_None)
			: Model.RootBone;
		const FTransform RootComponentTransform(SkeletalMesh->GetComposedRefPoseMatrix(RootBoneName));

		for (FAircraftRotorDefinition& Rotor : Model.Rotors)
		{
			if (!Rotor.bUseSocketTransform || Rotor.SocketName.IsNone())
			{
				continue;
			}

			// 存在性检查：名字既不是骨骼也不是 socket 时跳过（GetComposedRefPoseMatrix
			// 对不存在的名字返回 Identity，无法与"在原点的骨骼"区分）。
			const bool bIsBone = ReferenceSkeleton.FindBoneIndex(Rotor.SocketName) != INDEX_NONE;
			const bool bIsSocket = SkeletalMesh->FindSocket(Rotor.SocketName) != nullptr;
			if (!bIsBone && !bIsSocket)
			{
				continue;
			}

			const FTransform SocketBodyTransform(
				FMatrix(SkeletalMesh->GetComposedRefPoseMatrix(Rotor.SocketName))
				* RootComponentTransform.ToMatrixWithScale().Inverse());
			Rotor.PositionLocalCm = SocketBodyTransform.GetTranslation();
		}
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
	const FString SkeletalMeshKey = SourceSkeletalMesh
		? SourceSkeletalMesh->GetOutermost()->GetPersistentGuid().ToString() : TEXT("None");
	const FString PhysicsAssetKey = PhysicsAsset
		? PhysicsAsset->GetOutermost()->GetPersistentGuid().ToString() : TEXT("None");

	TArray<uint8> SerializedCollections;
	FMemoryWriter Writer(SerializedCollections, /*bIsPersistent=*/true);
	SerializeCollections(Writer,
		const_cast<UAircraftAsset*>(this)->GetAircraftCollectionsInternal());
	const FString CollectionHash = FMD5::HashBytes(
		SerializedCollections.GetData(), SerializedCollections.Num());

	return FString::Printf(TEXT("AircraftAsset_%d_%s_%s_%s"),
		FAircraftAssetCustomVersion::LatestVersion,
		*CollectionHash,
		*SkeletalMeshKey,
		*PhysicsAssetKey);
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
	if (ErrorText)
	{
		*ErrorText = FText::GetEmpty();
	}
	if (VerboseText)
	{
		*VerboseText = FText::GetEmpty();
	}

	auto AppendValidationError = [ErrorText, VerboseText](int32 LodIndex, const FText& ValidationError)
	{
		if (ErrorText)
		{
			*ErrorText = LOCTEXT("BuildErrorText", "Aircraft asset build failed validation.");
		}
		if (VerboseText)
		{
			const FText FormattedError = FText::Format(
				LOCTEXT("BuildValidationError", "LOD {0}: {1}"), LodIndex, ValidationError);
			*VerboseText = VerboseText->IsEmpty()
				? FormattedError
				: FText::Format(LOCTEXT("AppendBuildValidationError", "{0}\n{1}"), *VerboseText, FormattedError);
		}
	};

	// 不做静态 schema 校验（与 Terminal 节点一致）：
	// 每级 LOD 需要哪些组由它的 DriveMode 在运行时自行消费，缺失组走默认值
	// （FAircraftSimulationModel 构建对缺组本就容错）。图的正确性由作者人为控制
	// —— Frame→Constraint→LOD1 这类"按需求挂载"的精简支路是合法拓扑。
	if (InAircraftCollections.IsEmpty())
	{
		AppendValidationError(0, LOCTEXT("MissingAircraftCollection", "At least one aircraft collection is required."));
		return;
	}

	TArray<TSharedRef<const FManagedArrayCollection>> BuiltAircraftCollections;
	BuiltAircraftCollections.Reserve(InAircraftCollections.Num());

	for (int32 LodIndex = 0; LodIndex < InAircraftCollections.Num(); ++LodIndex)
	{
		TSharedRef<FManagedArrayCollection> AircraftCollection = MakeShared<FManagedArrayCollection>(*InAircraftCollections[LodIndex]);
		FAircraftCollection AircraftFacade(AircraftCollection);
		AircraftFacade.DefineSchema();
		BuiltAircraftCollections.Emplace(MoveTemp(AircraftCollection));
	}

	GetAircraftCollectionsInternal() = MoveTemp(BuiltAircraftCollections);

	EnsureCollectionsInitialized();
	SynchronizeAssetStateFromCollections();
	OnAssetChanged();
}

void UAircraftAsset::Serialize(FArchive& Ar)
{
	Ar.UsingCustomVersion(FAircraftAssetCustomVersion::GUID);
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
	return AircraftSimulationModel.IsValid()
		&& AircraftSimulationModel->SkeletalMesh != nullptr
		&& AircraftSimulationModel->GetNumLods() > 0;
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
		AircraftSimulationModel->SkeletalMesh = ResolveSourceSkeletalMesh(GetAircraftCollections());
		AircraftSimulationModel->PhysicsAsset = PhysicsAsset;
		for (FAircraftSimulationLodModel& LodModel : AircraftSimulationModel->LodModels)
		{
			ResolveRotorSocketTransforms(LodModel, AircraftSimulationModel->SkeletalMesh);
		}
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

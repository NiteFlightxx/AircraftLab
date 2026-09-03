#include "AircraftAsset/AircraftAssetBase.h"

#if WITH_EDITORONLY_DATA
#include "Animation/AnimationAsset.h"
#endif
#include "Engine/SkeletalMesh.h"
#include "PhysicsEngine/PhysicsAsset.h"
#include "ReferenceSkeleton.h"
#include "AircraftAsset/AircraftComponent.h"

namespace
{
	TArray<FMatrix44f>& GetEmptyRefBasesInvMatrix()
	{
		static TArray<FMatrix44f> EmptyRefBasesInvMatrix;
		return EmptyRefBasesInvMatrix;
	}

	const TArray<FMatrix44f>& GetEmptyRefBasesInvMatrixConst()
	{
		return GetEmptyRefBasesInvMatrix();
	}

	TArray<USkeletalMeshSocket*>& GetEmptySocketList()
	{
		static TArray<USkeletalMeshSocket*> EmptySocketList;
		return EmptySocketList;
	}

	const TArray<USkeletalMeshSocket*>& GetEmptySocketListConst()
	{
		return GetEmptySocketList();
	}

	const FPerPlatformInt& GetDefaultMinLodSetting()
	{
		static const FPerPlatformInt DefaultMinLod(0);
		return DefaultMinLod;
	}

	const FPerPlatformBool& GetDefaultDisableBelowMinLodStripping()
	{
		static const FPerPlatformBool DefaultDisableBelowMinLodStripping(false);
		return DefaultDisableBelowMinLodStripping;
	}
}

UAircraftAssetBase::UAircraftAssetBase(const FObjectInitializer& ObjectInitializer)
	: Super(ObjectInitializer)
	, DataflowInstance(this)
	, FallbackRefSkeleton(CreateFallbackReferenceSkeleton())
{
}

TObjectPtr<UDataflowBaseContent> UAircraftAssetBase::CreateDataflowContent()
{
	TObjectPtr<UDataflowSkeletalContent> SkeletalContent =
		UE::DataflowContextHelpers::CreateNewDataflowContent<UDataflowSkeletalContent>(this);

	SkeletalContent->SetDataflowOwner(this);
	SkeletalContent->SetTerminalAsset(this);

	WriteDataflowContent(SkeletalContent);

	SkeletalContent->OnContentDataChanged.AddUObject(this, &UAircraftAssetBase::UpdateSimulationActor);

	return SkeletalContent;
}

void UAircraftAssetBase::WriteDataflowContent(const TObjectPtr<UDataflowBaseContent>& DataflowContent) const
{
	if (const TObjectPtr<UDataflowSkeletalContent> SkeletalContent = Cast<UDataflowSkeletalContent>(DataflowContent))
	{
		SkeletalContent->SetDataflowAsset(GetDataflowInstance().GetDataflowAsset());
		SkeletalContent->SetDataflowTerminal(GetDataflowInstance().GetDataflowTerminal().ToString());

#if WITH_EDITORONLY_DATA
		SkeletalContent->SetAnimationAsset(GetPreviewSceneAnimation());
		SkeletalContent->SetSkeletalMesh(GetPreviewSceneSkeletalMesh());
#endif
	}
}

void UAircraftAssetBase::ReadDataflowContent(const TObjectPtr<UDataflowBaseContent>& DataflowContent)
{
	if (const TObjectPtr<UDataflowSkeletalContent> SkeletalContent = Cast<UDataflowSkeletalContent>(DataflowContent))
	{
#if WITH_EDITORONLY_DATA
		PreviewSceneAnimation = SkeletalContent->GetAnimationAsset();
		PreviewSceneSkeletalMesh = SkeletalContent->GetSkeletalMesh();
#endif
	}
}

const FDataflowInstance& UAircraftAssetBase::GetDataflowInstance() const
{
	return DataflowInstance;
}

FDataflowInstance& UAircraftAssetBase::GetDataflowInstance()
{
	return DataflowInstance;
}

void UAircraftAssetBase::PostLoad()
{
	Super::PostLoad();

#if WITH_EDITORONLY_DATA
	bHasDataflowAsset = (GetDataflowInstance().GetDataflowAsset() != nullptr);
#endif
}

#if WITH_EDITOR
void UAircraftAssetBase::PostEditChangeProperty(FPropertyChangedEvent& PropertyChangedEvent)
{
	if (PropertyChangedEvent.GetMemberPropertyName() == GET_MEMBER_NAME_CHECKED(UAircraftAssetBase, DataflowInstance) &&
		PropertyChangedEvent.GetPropertyName() == FName(TEXT("DataflowAsset")))
	{
#if WITH_EDITORONLY_DATA
		bHasDataflowAsset = (GetDataflowInstance().GetDataflowAsset() != nullptr);
#endif
	}

	InvalidateDataflowContents();

	Super::PostEditChangeProperty(PropertyChangedEvent);
}
#endif

void UAircraftAssetBase::SetDataflow(UDataflow* InDataflow)
{
	DataflowInstance.SetDataflowAsset(InDataflow);

#if WITH_EDITORONLY_DATA
	bHasDataflowAsset = (GetDataflowInstance().GetDataflowAsset() != nullptr);
#endif
}

UDataflow* UAircraftAssetBase::GetDataflow()
{
	return DataflowInstance.GetDataflowAsset();
}

const UDataflow* UAircraftAssetBase::GetDataflow() const
{
	return DataflowInstance.GetDataflowAsset();
}

FReferenceSkeleton& UAircraftAssetBase::GetRefSkeleton()
{
	if (const USkeletalMesh* SourceSkeletalMesh = GetSourceSkeletalMesh())
	{
		return const_cast<FReferenceSkeleton&>(SourceSkeletalMesh->GetRefSkeleton());
	}

	return FallbackRefSkeleton;
}

const FReferenceSkeleton& UAircraftAssetBase::GetRefSkeleton() const
{
	if (const USkeletalMesh* SourceSkeletalMesh = GetSourceSkeletalMesh())
	{
		return SourceSkeletalMesh->GetRefSkeleton();
	}

	return FallbackRefSkeleton;
}

FSkeletalMeshLODInfo* UAircraftAssetBase::GetLODInfo(int32 Index)
{
	if (USkeletalMesh* SourceSkeletalMesh = GetSourceSkeletalMesh())
	{
		return SourceSkeletalMesh->GetLODInfo(Index);
	}

	return nullptr;
}

const FSkeletalMeshLODInfo* UAircraftAssetBase::GetLODInfo(int32 Index) const
{
	if (const USkeletalMesh* SourceSkeletalMesh = GetSourceSkeletalMesh())
	{
		return SourceSkeletalMesh->GetLODInfo(Index);
	}

	return nullptr;
}

UPhysicsAsset* UAircraftAssetBase::GetShadowPhysicsAsset() const
{
	return GetPhysicsAsset();
}

FMatrix UAircraftAssetBase::GetComposedRefPoseMatrix(FName BoneName) const
{
	if (const USkeletalMesh* SourceSkeletalMesh = GetSourceSkeletalMesh())
	{
		return SourceSkeletalMesh->GetComposedRefPoseMatrix(BoneName);
	}

	const int32 BoneIndex = GetRefSkeleton().FindBoneIndex(BoneName);
	return GetComposedRefPoseMatrix(BoneIndex);
}

FMatrix UAircraftAssetBase::GetComposedRefPoseMatrix(int32 BoneIndex) const
{
	if (const USkeletalMesh* SourceSkeletalMesh = GetSourceSkeletalMesh())
	{
		return SourceSkeletalMesh->GetComposedRefPoseMatrix(BoneIndex);
	}

	return FMatrix::Identity;
}

const FMeshUVChannelInfo* UAircraftAssetBase::GetUVChannelData(int32 MaterialIndex) const
{
	if (const USkeletalMesh* SourceSkeletalMesh = GetSourceSkeletalMesh())
	{
		return SourceSkeletalMesh->GetUVChannelData(MaterialIndex);
	}

	return nullptr;
}

bool UAircraftAssetBase::GetSupportRayTracing() const
{
	if (const USkeletalMesh* SourceSkeletalMesh = GetSourceSkeletalMesh())
	{
		return SourceSkeletalMesh->GetSupportRayTracing();
	}

	return false;
}

int32 UAircraftAssetBase::GetRayTracingMinLOD() const
{
	if (const USkeletalMesh* SourceSkeletalMesh = GetSourceSkeletalMesh())
	{
		return SourceSkeletalMesh->GetRayTracingMinLOD();
	}

	return 0;
}

TArray<FMatrix44f>& UAircraftAssetBase::GetRefBasesInvMatrix()
{
	if (USkeletalMesh* SourceSkeletalMesh = GetSourceSkeletalMesh())
	{
		return SourceSkeletalMesh->GetRefBasesInvMatrix();
	}

	return GetEmptyRefBasesInvMatrix();
}

const TArray<FMatrix44f>& UAircraftAssetBase::GetRefBasesInvMatrix() const
{
	if (const USkeletalMesh* SourceSkeletalMesh = GetSourceSkeletalMesh())
	{
		return SourceSkeletalMesh->GetRefBasesInvMatrix();
	}

	return GetEmptyRefBasesInvMatrixConst();
}

TArray<FSkeletalMeshLODInfo>& UAircraftAssetBase::GetLODInfoArray()
{
	static TArray<FSkeletalMeshLODInfo> EmptyLodInfoArray;
	return EmptyLodInfoArray;
}

const TArray<FSkeletalMeshLODInfo>& UAircraftAssetBase::GetLODInfoArray() const
{
	static const TArray<FSkeletalMeshLODInfo> EmptyLodInfoArray;
	return EmptyLodInfoArray;
}

FSkeletalMeshRenderData* UAircraftAssetBase::GetResourceForRendering() const
{
	if (const USkeletalMesh* SourceSkeletalMesh = GetSourceSkeletalMesh())
	{
		return SourceSkeletalMesh->GetResourceForRendering();
	}
	return nullptr;
}

int32 UAircraftAssetBase::GetDefaultMinLod() const
{
	if (const USkeletalMesh* SourceSkeletalMesh = GetSourceSkeletalMesh())
	{
		return SourceSkeletalMesh->GetDefaultMinLod();
	}

	return 0;
}

const FPerPlatformInt& UAircraftAssetBase::GetMinLod() const
{
	if (const USkeletalMesh* SourceSkeletalMesh = GetSourceSkeletalMesh())
	{
		return SourceSkeletalMesh->GetMinLod();
	}

	return GetDefaultMinLodSetting();
}

TArray<FSkeletalMaterial>& UAircraftAssetBase::GetMaterials()
{
	if (USkeletalMesh* SourceSkeletalMesh = GetSourceSkeletalMesh())
	{
		return SourceSkeletalMesh->GetMaterials();
	}

	static TArray<FSkeletalMaterial> EmptyMaterials;
	return EmptyMaterials;
}

const TArray<FSkeletalMaterial>& UAircraftAssetBase::GetMaterials() const
{
	if (const USkeletalMesh* SourceSkeletalMesh = GetSourceSkeletalMesh())
	{
		return SourceSkeletalMesh->GetMaterials();
	}

	static const TArray<FSkeletalMaterial> EmptyMaterials;
	return EmptyMaterials;
}

int32 UAircraftAssetBase::GetLODNum() const
{
	if (const USkeletalMesh* SourceSkeletalMesh = GetSourceSkeletalMesh())
	{
		return SourceSkeletalMesh->GetLODNum();
	}

	return 0;
}

bool UAircraftAssetBase::IsMaterialUsed(int32 MaterialIndex) const
{
	if (const USkeletalMesh* SourceSkeletalMesh = GetSourceSkeletalMesh())
	{
		return SourceSkeletalMesh->IsMaterialUsed(MaterialIndex);
	}

	return false;
}

FBoxSphereBounds UAircraftAssetBase::GetBounds() const
{
	if (const USkeletalMesh* SourceSkeletalMesh = GetSourceSkeletalMesh())
	{
		return SourceSkeletalMesh->GetBounds();
	}

	return FBoxSphereBounds(EForceInit::ForceInit);
}

TArray<USkeletalMeshSocket*> UAircraftAssetBase::GetActiveSocketList() const
{
	if (const USkeletalMesh* SourceSkeletalMesh = GetSourceSkeletalMesh())
	{
		return SourceSkeletalMesh->GetActiveSocketList();
	}

	return GetEmptySocketListConst();
}

USkeletalMeshSocket* UAircraftAssetBase::FindSocket(FName InSocketName) const
{
	if (const USkeletalMesh* SourceSkeletalMesh = GetSourceSkeletalMesh())
	{
		return SourceSkeletalMesh->FindSocket(InSocketName);
	}

	return nullptr;
}

USkeletalMeshSocket* UAircraftAssetBase::FindSocketInfo(FName InSocketName, FTransform& OutTransform, int32& OutBoneIndex, int32& OutIndex) const
{
	if (const USkeletalMesh* SourceSkeletalMesh = GetSourceSkeletalMesh())
	{
		return SourceSkeletalMesh->FindSocketInfo(InSocketName, OutTransform, OutBoneIndex, OutIndex);
	}

	return nullptr;
}

UMeshDeformer* UAircraftAssetBase::GetDefaultMeshDeformer() const
{
	if (const USkeletalMesh* SourceSkeletalMesh = GetSourceSkeletalMesh())
	{
		return SourceSkeletalMesh->GetDefaultMeshDeformer();
	}

	return nullptr;
}

UMeshDeformerCollection* UAircraftAssetBase::GetTargetMeshDeformers() const
{
	if (const USkeletalMesh* SourceSkeletalMesh = GetSourceSkeletalMesh())
	{
		return SourceSkeletalMesh->GetTargetMeshDeformers();
	}

	return nullptr;
}

bool UAircraftAssetBase::HasHalfEdgeBuffer(int32 LODIndex) const
{
	if (const USkeletalMesh* SourceSkeletalMesh = GetSourceSkeletalMesh())
	{
		return SourceSkeletalMesh->HasHalfEdgeBuffer(LODIndex);
	}

	return false;
}

UMaterialInterface* UAircraftAssetBase::GetOverlayMaterial() const
{
	if (const USkeletalMesh* SourceSkeletalMesh = GetSourceSkeletalMesh())
	{
		return SourceSkeletalMesh->GetOverlayMaterial();
	}

	return nullptr;
}

float UAircraftAssetBase::GetOverlayMaterialMaxDrawDistance() const
{
	if (const USkeletalMesh* SourceSkeletalMesh = GetSourceSkeletalMesh())
	{
		return SourceSkeletalMesh->GetOverlayMaterialMaxDrawDistance();
	}

	return 0.0f;
}

bool UAircraftAssetBase::IsValidLODIndex(int32 Index) const
{
	if (const USkeletalMesh* SourceSkeletalMesh = GetSourceSkeletalMesh())
	{
		return SourceSkeletalMesh->IsValidLODIndex(Index);
	}

	return false;
}

int32 UAircraftAssetBase::GetMinLodIdx(bool bForceLowestLODIdx) const
{
	if (const USkeletalMesh* SourceSkeletalMesh = GetSourceSkeletalMesh())
	{
		return SourceSkeletalMesh->GetMinLodIdx(bForceLowestLODIdx);
	}

	return 0;
}

bool UAircraftAssetBase::NeedCPUData(int32 LODIndex) const
{
	if (const USkeletalMesh* SourceSkeletalMesh = GetSourceSkeletalMesh())
	{
		return SourceSkeletalMesh->NeedCPUData(LODIndex);
	}

	return false;
}

bool UAircraftAssetBase::GetHasVertexColors() const
{
	if (const USkeletalMesh* SourceSkeletalMesh = GetSourceSkeletalMesh())
	{
		return SourceSkeletalMesh->GetHasVertexColors();
	}

	return false;
}

int32 UAircraftAssetBase::GetPlatformMinLODIdx(const ITargetPlatform* TargetPlatform) const
{
	if (const USkeletalMesh* SourceSkeletalMesh = GetSourceSkeletalMesh())
	{
		return SourceSkeletalMesh->GetPlatformMinLODIdx(TargetPlatform);
	}

	return 0;
}

const FPerPlatformBool& UAircraftAssetBase::GetDisableBelowMinLodStripping() const
{
	if (const USkeletalMesh* SourceSkeletalMesh = GetSourceSkeletalMesh())
	{
		return SourceSkeletalMesh->GetDisableBelowMinLodStripping();
	}

	return GetDefaultDisableBelowMinLodStripping();
}

#if WITH_EDITORONLY_DATA
void UAircraftAssetBase::SetPreviewSceneSkeletalMesh(USkeletalMesh* Mesh)
{
	PreviewSceneSkeletalMesh = Mesh;
}

USkeletalMesh* UAircraftAssetBase::GetPreviewSceneSkeletalMesh() const
{
	return PreviewSceneSkeletalMesh.LoadSynchronous();
}

void UAircraftAssetBase::SetPreviewSceneAnimation(UAnimationAsset* Animation)
{
	PreviewSceneAnimation = Animation;
}

UAnimationAsset* UAircraftAssetBase::GetPreviewSceneAnimation() const
{
	return PreviewSceneAnimation.LoadSynchronous();
}
#endif

void UAircraftAssetBase::OnPropertyChanged() const
{
}

void UAircraftAssetBase::OnAssetChanged() const
{
	for (UAircraftComponent* const Component : GetDependentComponents())
	{
		if (IsValid(Component))
		{
			Component->RefreshAssetState();
		}
	}
}

void UAircraftAssetBase::UpdateSimulationActor(TObjectPtr<AActor>& SimulationActor) const
{
	TInlineComponentArray<UAircraftComponent*> AircraftComponents(SimulationActor);
	for (UAircraftComponent* Component : AircraftComponents)
	{
		if (Component->GetAsset() == this)
		{
#if WITH_EDITOR
			Component->RefreshAssetState();
#endif
		}
		else if (!Component->GetAsset())
		{
			Component->SetAsset(const_cast<UAircraftAssetBase*>(this));
		}
	}
}

TArray<UAircraftComponent*> UAircraftAssetBase::GetDependentComponents() const
{
	TArray<UAircraftComponent*> DependentComponents;

	for (TObjectIterator<UAircraftComponent> ObjectIterator; ObjectIterator; ++ObjectIterator)
	{
		if (UAircraftComponent* const Component = *ObjectIterator)
		{
			if (Component->GetAsset() == this)
			{
				DependentComponents.Emplace(Component);
			}
		}
	}

	return DependentComponents;
}

FReferenceSkeleton UAircraftAssetBase::CreateFallbackReferenceSkeleton()
{
	FReferenceSkeleton ReferenceSkeleton;
	FReferenceSkeletonModifier ReferenceSkeletonModifier(ReferenceSkeleton, nullptr);

	FMeshBoneInfo RootBoneInfo;
	constexpr TCHAR RootBoneName[] = TEXT("Root");
	RootBoneInfo.ParentIndex = INDEX_NONE;
#if WITH_EDITORONLY_DATA
	RootBoneInfo.ExportName = RootBoneName;
#endif
	RootBoneInfo.Name = FName(RootBoneName);
	ReferenceSkeletonModifier.Add(RootBoneInfo, FTransform::Identity);

	return ReferenceSkeleton;
}

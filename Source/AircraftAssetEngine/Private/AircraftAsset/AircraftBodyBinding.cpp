#include "AircraftAsset/AircraftBodyBinding.h"

#include "Engine/SkeletalMesh.h"
#include "PhysicsEngine/PhysicsAsset.h"

FName UE::AircraftLab::AircraftAsset::ResolveAircraftChassisBodyName(
	const USkeletalMesh* const SkeletalMesh,
	const UPhysicsAsset* const PhysicsAsset,
	const FName ConfiguredRootBone)
{
	if (!SkeletalMesh || !PhysicsAsset)
	{
		return NAME_None;
	}
	if (!ConfiguredRootBone.IsNone())
	{
		return PhysicsAsset->FindBodyIndex(ConfiguredRootBone) != INDEX_NONE
			? ConfiguredRootBone
			: NAME_None;
	}
	const FReferenceSkeleton& ReferenceSkeleton = SkeletalMesh->GetRefSkeleton();
	for (int32 BoneIndex = 0; BoneIndex < ReferenceSkeleton.GetNum(); ++BoneIndex)
	{
		const FName BoneName = ReferenceSkeleton.GetBoneName(BoneIndex);
		if (PhysicsAsset->FindBodyIndex(BoneName) != INDEX_NONE)
		{
			return BoneName;
		}
	}
	return NAME_None;
}

#pragma once

#include "CoreMinimal.h"
#include "BoneContainer.h"
#include "BonePose.h"
#include "AircraftAsset/AircraftAnimInstance.h"
#include "BoneControllers/AnimNode_SkeletalControlBase.h"

#include "AnimNode_AircraftWheelController.generated.h"

USTRUCT()
struct AIRCRAFTASSETENGINE_API FAnimNode_AircraftWheelController : public FAnimNode_SkeletalControlBase
{
	GENERATED_BODY()

	FAnimNode_AircraftWheelController();

	virtual void GatherDebugData(FNodeDebugData& DebugData) override;
	virtual void EvaluateSkeletalControl_AnyThread(FComponentSpacePoseContext& Output, TArray<FBoneTransform>& OutBoneTransforms) override;
	virtual bool IsValidToEvaluate(const USkeleton* Skeleton, const FBoneContainer& RequiredBones) override;
	virtual void Initialize_AnyThread(const FAnimationInitializeContext& Context) override;

private:
	virtual void InitializeBoneReferences(const FBoneContainer& RequiredBones) override;

	void RebuildWheelLookup(const FBoneContainer& RequiredBones);

	struct FWheelLookupData
	{
		int32 WheelIndex = INDEX_NONE;
		FBoneReference BoneReference;
	};

	TArray<FWheelLookupData> Wheels;
	const FAircraftAnimInstanceProxy* AircraftAnimInstanceProxy = nullptr;
};

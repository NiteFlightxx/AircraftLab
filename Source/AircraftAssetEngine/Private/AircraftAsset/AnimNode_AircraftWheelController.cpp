#include "AircraftAsset/AnimNode_AircraftWheelController.h"

#include "AnimationRuntime.h"

#include UE_INLINE_GENERATED_CPP_BY_NAME(AnimNode_AircraftWheelController)

FAnimNode_AircraftWheelController::FAnimNode_AircraftWheelController() = default;

void FAnimNode_AircraftWheelController::GatherDebugData(FNodeDebugData& DebugData)
{
	FString DebugLine = DebugData.GetNodeName(this);
	DebugLine += TEXT("(Aircraft Wheel Controller)");
	DebugData.AddDebugItem(DebugLine);

	const TArray<FAircraftWheelAnimationData>* WheelAnimationData = AircraftAnimInstanceProxy
		? &AircraftAnimInstanceProxy->GetWheelAnimationData()
		: nullptr;

	for (const FWheelLookupData& Wheel : Wheels)
	{
		if (!WheelAnimationData || !WheelAnimationData->IsValidIndex(Wheel.WheelIndex))
		{
			DebugData.AddDebugItem(FString::Printf(TEXT("Wheel %d : no animation data"), Wheel.WheelIndex));
			continue;
		}

		const FAircraftWheelAnimationData& WheelData = (*WheelAnimationData)[Wheel.WheelIndex];
		DebugData.AddDebugItem(FString::Printf(
			TEXT("Wheel %d Bone %s Rot %s Loc %s"),
			Wheel.WheelIndex,
			*WheelData.BoneName.ToString(),
			*WheelData.RotOffset.ToString(),
			*WheelData.LocOffset.ToString()));
	}

	ComponentPose.GatherDebugData(DebugData);
}

void FAnimNode_AircraftWheelController::EvaluateSkeletalControl_AnyThread(
	FComponentSpacePoseContext& Output,
	TArray<FBoneTransform>& OutBoneTransforms)
{
	check(OutBoneTransforms.Num() == 0);

	if (!AircraftAnimInstanceProxy)
	{
		return;
	}

	const TArray<FAircraftWheelAnimationData>& WheelAnimationData = AircraftAnimInstanceProxy->GetWheelAnimationData();
	const FBoneContainer& BoneContainer = Output.Pose.GetPose().GetBoneContainer();

	if (Wheels.Num() != WheelAnimationData.Num())
	{
		RebuildWheelLookup(BoneContainer);
	}

	for (const FWheelLookupData& Wheel : Wheels)
	{
		if (!Wheel.BoneReference.IsValidToEvaluate(BoneContainer) ||
			!WheelAnimationData.IsValidIndex(Wheel.WheelIndex))
		{
			continue;
		}

		const FAircraftWheelAnimationData& WheelData = WheelAnimationData[Wheel.WheelIndex];
		const FCompactPoseBoneIndex WheelBoneIndex = Wheel.BoneReference.GetCompactPoseIndex(BoneContainer);

		FTransform NewBoneTM = Output.Pose.GetComponentSpaceTransform(WheelBoneIndex);
		FAnimationRuntime::ConvertCSTransformToBoneSpace(
			Output.AnimInstanceProxy->GetComponentTransform(),
			Output.Pose,
			NewBoneTM,
			WheelBoneIndex,
			BCS_ComponentSpace);

		const FQuat BoneRotationOffset(WheelData.RotOffset);
		NewBoneTM.SetRotation(BoneRotationOffset * NewBoneTM.GetRotation());
		NewBoneTM.AddToTranslation(WheelData.LocOffset);

		FAnimationRuntime::ConvertBoneSpaceTransformToCS(
			Output.AnimInstanceProxy->GetComponentTransform(),
			Output.Pose,
			NewBoneTM,
			WheelBoneIndex,
			BCS_ComponentSpace);

		OutBoneTransforms.Add(FBoneTransform(WheelBoneIndex, NewBoneTM));
	}
}

bool FAnimNode_AircraftWheelController::IsValidToEvaluate(const USkeleton* Skeleton, const FBoneContainer& RequiredBones)
{
	(void)Skeleton;

	for (const FWheelLookupData& Wheel : Wheels)
	{
		if (Wheel.BoneReference.IsValidToEvaluate(RequiredBones))
		{
			return true;
		}
	}

	return false;
}

void FAnimNode_AircraftWheelController::Initialize_AnyThread(const FAnimationInitializeContext& Context)
{
	FAnimNode_SkeletalControlBase::Initialize_AnyThread(Context);
	AircraftAnimInstanceProxy = static_cast<const FAircraftAnimInstanceProxy*>(Context.AnimInstanceProxy);
}

void FAnimNode_AircraftWheelController::InitializeBoneReferences(const FBoneContainer& RequiredBones)
{
	RebuildWheelLookup(RequiredBones);
}

void FAnimNode_AircraftWheelController::RebuildWheelLookup(const FBoneContainer& RequiredBones)
{
	Wheels.Reset();

	if (!AircraftAnimInstanceProxy)
	{
		return;
	}

	const TArray<FAircraftWheelAnimationData>& WheelAnimationData = AircraftAnimInstanceProxy->GetWheelAnimationData();
	Wheels.Reserve(WheelAnimationData.Num());

	for (int32 WheelIndex = 0; WheelIndex < WheelAnimationData.Num(); ++WheelIndex)
	{
		const FAircraftWheelAnimationData& WheelData = WheelAnimationData[WheelIndex];
		if (WheelData.BoneName.IsNone())
		{
			continue;
		}

		FWheelLookupData& WheelLookupData = Wheels.AddDefaulted_GetRef();
		WheelLookupData.WheelIndex = WheelIndex;
		WheelLookupData.BoneReference.BoneName = WheelData.BoneName;
		WheelLookupData.BoneReference.Initialize(RequiredBones);
	}

	Wheels.Sort([](const FWheelLookupData& Left, const FWheelLookupData& Right)
	{
		return Left.BoneReference.BoneIndex < Right.BoneReference.BoneIndex;
	});
}

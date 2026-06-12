#include "Dataflow/AircraftSuspensionConfigNode.h"

#include "AircraftAsset/CollectionAircraftConstFacade.h"

#include UE_INLINE_GENERATED_CPP_BY_NAME(AircraftSuspensionConfigNode)

namespace UE::AircraftLab::AircraftAssetDataflowNodes::Private
{
	static int32 FindSuspensionIndex(
		const TManagedArray<FName>& SuspensionNames,
		const FName SuspensionName)
	{
		for (int32 Index = 0; Index < SuspensionNames.Num(); ++Index)
		{
			if (SuspensionNames[Index] == SuspensionName)
			{
				return Index;
			}
		}

		return INDEX_NONE;
	}
}

FAircraftSuspensionConfigNode::FAircraftSuspensionConfigNode(const UE::Dataflow::FNodeParameters& InParam, FGuid InGuid)
	: FAircraftConfigNodeBase(InParam, InGuid)
{
	RegisterAircraftConnections();
	RegisterInputConnection(&bReplaceAllSuspensions);
	RegisterInputConnection(&SuspensionName);
	RegisterInputConnection(&TopMountLocal);
	RegisterInputConnection(&LowerBallJointLocal);
	RegisterInputConnection(&MaxRaiseCm);
	RegisterInputConnection(&MaxDropCm);
	RegisterInputConnection(&NaturalFrequencyHz);
	RegisterInputConnection(&DampingRatio);
}

void FAircraftSuspensionConfigNode::AddProperties(FPropertyHelper& /*PropertyHelper*/) const
{
}

void FAircraftSuspensionConfigNode::EvaluateAircraftCollection(
	UE::Dataflow::FContext& /*Context*/,
	const TSharedRef<FManagedArrayCollection>& AircraftCollection,
	FAircraftConfigNodeBase::FAircraftFacade& InFacade) const
{
	using namespace UE::AircraftLab::AircraftAsset;

	if (bReplaceAllSuspensions)
	{
		AircraftCollection->Resize(0, AircraftCollectionGroup::Suspensions);
	}

	if (SuspensionName.IsNone())
	{
		return;
	}

	TManagedArray<FName>& SuspensionNames = InFacade.FindOrAddAttribute<FName>(AircraftCollectionAttribute::SuspensionName, AircraftCollectionGroup::Suspensions);

	int32 SuspensionIndex = UE::AircraftLab::AircraftAssetDataflowNodes::Private::FindSuspensionIndex(SuspensionNames, SuspensionName);
	if (SuspensionIndex == INDEX_NONE)
	{
		SuspensionIndex = AircraftCollection->AddElements(1, AircraftCollectionGroup::Suspensions);
	}

	SuspensionNames[SuspensionIndex] = SuspensionName;
	InFacade.FindOrAddAttribute<FVector3f>(AircraftCollectionAttribute::SuspensionTopMountLocal, AircraftCollectionGroup::Suspensions)[SuspensionIndex] = FVector3f(TopMountLocal);
	InFacade.FindOrAddAttribute<FVector3f>(AircraftCollectionAttribute::SuspensionLowerBallJointLocal, AircraftCollectionGroup::Suspensions)[SuspensionIndex] = FVector3f(LowerBallJointLocal);
	InFacade.FindOrAddAttribute<float>(AircraftCollectionAttribute::SuspensionMaxRaiseCm, AircraftCollectionGroup::Suspensions)[SuspensionIndex] = MaxRaiseCm;
	InFacade.FindOrAddAttribute<float>(AircraftCollectionAttribute::SuspensionMaxDropCm, AircraftCollectionGroup::Suspensions)[SuspensionIndex] = MaxDropCm;
	InFacade.FindOrAddAttribute<float>(AircraftCollectionAttribute::SuspensionNaturalFrequencyHz, AircraftCollectionGroup::Suspensions)[SuspensionIndex] = NaturalFrequencyHz;
	InFacade.FindOrAddAttribute<float>(AircraftCollectionAttribute::SuspensionDampingRatio, AircraftCollectionGroup::Suspensions)[SuspensionIndex] =
		DampingRatio;
}

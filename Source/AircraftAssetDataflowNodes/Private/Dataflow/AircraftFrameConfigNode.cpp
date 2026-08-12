#include "Dataflow/AircraftFrameConfigNode.h"

#include "AircraftAsset/AircraftCollection.h"
#include "AircraftAsset/CollectionAircraftConstFacade.h"
#include "AircraftAsset/CollectionAircraftPropertyFacade.h"

#include UE_INLINE_GENERATED_CPP_BY_NAME(AircraftFrameConfigNode)

FAircraftFrameConfigNode::FAircraftFrameConfigNode(const UE::Dataflow::FNodeParameters& InParam, FGuid InGuid)
	: FDataflowNode(InParam, InGuid)
{
	RegisterInputConnection(&Collection);
	RegisterOutputConnection(&Collection, &Collection);
}

void FAircraftFrameConfigNode::Evaluate(UE::Dataflow::FContext& Context, const FDataflowOutput* Out) const
{
	using namespace UE::AircraftLab::AircraftAsset;

	if (!Out || !Out->IsA<FManagedArrayCollection>(&Collection))
	{
		return;
	}

	FManagedArrayCollection InCollection = GetValue<FManagedArrayCollection>(Context, &Collection);
	const TSharedRef<FManagedArrayCollection> AircraftCollection = MakeShared<FManagedArrayCollection>(MoveTemp(InCollection));

	FCollectionAircraftFacade Facade(AircraftCollection);
	Facade.DefineSchema();

	// Frame 是单元素组，DefineSchema 已 AddElements(1)，直接写入第 0 行。
	if (TArrayView<FName> RootBoneArr = Facade.GetFrameRootBone(); RootBoneArr.Num() > 0)
	{
		RootBoneArr[0] = RootBone;
	}
	if (TArrayView<uint8> FrameTypeArr = Facade.GetFrameType(); FrameTypeArr.Num() > 0)
	{
		FrameTypeArr[0] = static_cast<uint8>(FrameType);
	}
	if (TArrayView<float> MassArr = Facade.GetFrameMassKg(); MassArr.Num() > 0)
	{
		MassArr[0] = MassKg;
	}
	if (TArrayView<FVector3f> ComArr = Facade.GetFrameCenterOfMassOffsetCm(); ComArr.Num() > 0)
	{
		ComArr[0] = CenterOfMassOffsetCm;
	}
	if (TArrayView<FVector3f> InertiaArr = Facade.GetFrameInertiaDiagonalKgCmSq(); InertiaArr.Num() > 0)
	{
		InertiaArr[0] = InertiaDiagonalKgCmSq;
	}
	FCollectionAircraftPropertyMutableFacade Properties(AircraftCollection);
	Properties.DefineSchema();
	int32 ForwardAxisIndex = Properties.GetKeyNameIndex(TEXT("Frame.ForwardAxis"));
	if (ForwardAxisIndex == INDEX_NONE)
	{
		ForwardAxisIndex = Properties.AddProperty(TEXT("Frame.ForwardAxis"), EAircraftCollectionPropertyFlags::Enabled);
	}
	Properties.SetValue(ForwardAxisIndex, static_cast<int32>(ForwardAxis));

	SetValue(Context, MoveTemp(*AircraftCollection), &Collection);
}

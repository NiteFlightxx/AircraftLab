#include "Dataflow/AircraftFrameConfigNode.h"

#include "AircraftAsset/AircraftCollection.h"
#include "AircraftAsset/CollectionAircraftConstFacade.h"

#include UE_INLINE_GENERATED_CPP_BY_NAME(AircraftFrameConfigNode)

FAircraftFrameConfigNode::FAircraftFrameConfigNode(const UE::Dataflow::FNodeParameters& InParam, FGuid InGuid)
	: FDataflowNode(InParam, InGuid)
{
	RegisterInputConnection(&Collection);
	RegisterOutputConnection(&Collection, &Collection);

	RegisterInputConnection(&RootBone);
	RegisterInputConnection(&MassKg);
	RegisterInputConnection(&CenterOfMassOffsetCm);
	RegisterInputConnection(&InertiaDiagonalKgCmSq);
	RegisterInputConnection(&LinearDragPerAxis);
	RegisterInputConnection(&AngularDragPerAxis);
	RegisterInputConnection(&WindVelocityCmPerSec);
	RegisterInputConnection(&GroundEffectStartHeightCm);
	RegisterInputConnection(&GroundEffectStrength);
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
		RootBoneArr[0] = GetValue(Context, &RootBone);
	}
	if (TArrayView<uint8> FrameTypeArr = Facade.GetFrameType(); FrameTypeArr.Num() > 0)
	{
		FrameTypeArr[0] = static_cast<uint8>(FrameType);
	}
	if (TArrayView<float> MassArr = Facade.GetFrameMassKg(); MassArr.Num() > 0)
	{
		MassArr[0] = GetValue(Context, &MassKg);
	}
	if (TArrayView<FVector3f> ComArr = Facade.GetFrameCenterOfMassOffsetCm(); ComArr.Num() > 0)
	{
		ComArr[0] = GetValue(Context, &CenterOfMassOffsetCm);
	}
	if (TArrayView<FVector3f> InertiaArr = Facade.GetFrameInertiaDiagonalKgCmSq(); InertiaArr.Num() > 0)
	{
		InertiaArr[0] = GetValue(Context, &InertiaDiagonalKgCmSq);
	}
	if (TArrayView<FVector3f> LinDragArr = Facade.GetFrameLinearDragPerAxis(); LinDragArr.Num() > 0)
	{
		LinDragArr[0] = GetValue(Context, &LinearDragPerAxis);
	}
	if (TArrayView<FVector3f> AngDragArr = Facade.GetFrameAngularDragPerAxis(); AngDragArr.Num() > 0)
	{
		AngDragArr[0] = GetValue(Context, &AngularDragPerAxis);
	}
	if (TArrayView<FVector3f> WindArr = Facade.GetFrameWindVelocityCmPerSec(); WindArr.Num() > 0)
	{
		WindArr[0] = GetValue(Context, &WindVelocityCmPerSec);
	}
	if (TArrayView<float> GeStartArr = Facade.GetFrameGroundEffectStartHeightCm(); GeStartArr.Num() > 0)
	{
		GeStartArr[0] = GetValue(Context, &GroundEffectStartHeightCm);
	}
	if (TArrayView<float> GeStrengthArr = Facade.GetFrameGroundEffectStrength(); GeStrengthArr.Num() > 0)
	{
		GeStrengthArr[0] = GetValue(Context, &GroundEffectStrength);
	}

	SetValue(Context, MoveTemp(*AircraftCollection), &Collection);
}

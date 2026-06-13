#include "Dataflow/AircraftMotorConfigNode.h"

#include "AircraftAsset/AircraftCollection.h"
#include "AircraftAsset/CollectionAircraftConstFacade.h"

#include UE_INLINE_GENERATED_CPP_BY_NAME(AircraftMotorConfigNode)

FAircraftMotorConfigNode::FAircraftMotorConfigNode(const UE::Dataflow::FNodeParameters& InParam, FGuid InGuid)
	: FDataflowNode(InParam, InGuid)
{
	RegisterInputConnection(&Collection);
	RegisterOutputConnection(&Collection, &Collection);
}

void FAircraftMotorConfigNode::Evaluate(UE::Dataflow::FContext& Context, const FDataflowOutput* Out) const
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

	// 调整 Motors 组到指定数量。AddElements 会刷新底层 ManagedArray 缓存。
	const int32 DesiredCount = Motors.Num();
	const int32 CurrentCount = AircraftCollection->NumElements(AircraftCollectionGroup::Motors);
	if (CurrentCount < DesiredCount)
	{
		Facade.AddElements(DesiredCount - CurrentCount, AircraftCollectionGroup::Motors);
	}
	else if (CurrentCount > DesiredCount)
	{
		AircraftCollection->Resize(DesiredCount, AircraftCollectionGroup::Motors);
		// Resize 后底层数组容量改变，需要刷新缓存以便后续 GetXxx 拿到正确的 ArrayView。
		FCollectionAircraftFacade(AircraftCollection).DefineSchema();
	}

	// 重新构建 Facade 以拿到最新数组视图（AddElements/Resize 后底层指针可能失效）。
	FCollectionAircraftFacade WriteFacade(AircraftCollection);

	TArrayView<FName> Names = WriteFacade.GetMotorName();
	TManagedArray<bool>* EnabledArr = WriteFacade.GetMotorEnabled();
	TArrayView<float> MinRpms = WriteFacade.GetMotorMinRpm();
	TArrayView<float> IdleRpms = WriteFacade.GetMotorIdleRpm();
	TArrayView<float> MaxRpms = WriteFacade.GetMotorMaxRpm();
	TArrayView<float> SpinUps = WriteFacade.GetMotorSpinUpTimeSeconds();
	TArrayView<float> SpinDowns = WriteFacade.GetMotorSpinDownTimeSeconds();
	TArrayView<float> Exponents = WriteFacade.GetMotorCommandExponent();
	TArrayView<float> Slews = WriteFacade.GetMotorMaxCommandSlewPerSecond();

	const int32 N = Motors.Num();
	for (int32 i = 0; i < N; ++i)
	{
		const FAircraftMotorEntry& E = Motors[i];
		if (i < Names.Num())     { Names[i] = E.Name; }
		if (EnabledArr && i < EnabledArr->Num()) { (*EnabledArr)[i] = E.bEnabled; }
		if (i < MinRpms.Num())   { MinRpms[i] = E.MinRpm; }
		if (i < IdleRpms.Num())  { IdleRpms[i] = E.IdleRpm; }
		if (i < MaxRpms.Num())   { MaxRpms[i] = E.MaxRpm; }
		if (i < SpinUps.Num())   { SpinUps[i] = E.SpinUpTimeSeconds; }
		if (i < SpinDowns.Num()) { SpinDowns[i] = E.SpinDownTimeSeconds; }
		if (i < Exponents.Num()) { Exponents[i] = E.CommandExponent; }
		if (i < Slews.Num())     { Slews[i] = E.MaxCommandSlewPerSecond; }
	}

	SetValue(Context, MoveTemp(*AircraftCollection), &Collection);
}

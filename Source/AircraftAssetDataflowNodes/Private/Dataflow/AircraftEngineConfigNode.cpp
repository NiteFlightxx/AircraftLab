#include "Dataflow/AircraftEngineConfigNode.h"
#include "AircraftAsset/CollectionAircraftConstFacade.h"
#include "Curves/RichCurve.h"

namespace UE::AircraftLab::AircraftAssetDataflowNodes::Private
{
	static FString SerializeRuntimeFloatCurve(const FRuntimeFloatCurve& Curve)
	{
		const FRichCurve* RichCurve = Curve.GetRichCurveConst();
		if (!RichCurve || RichCurve->Keys.IsEmpty())
		{
			return FString();
		}

		TArray<FString> Tokens;
		Tokens.Reserve(RichCurve->Keys.Num());

		for (const FRichCurveKey& Key : RichCurve->Keys)
		{
			Tokens.Add(FString::Printf(TEXT("%s:%s"), *LexToString(Key.Time), *LexToString(Key.Value)));
		}

		return FString::Join(Tokens, TEXT(","));
	}
}

#include UE_INLINE_GENERATED_CPP_BY_NAME(AircraftEngineConfigNode)

FAircraftEngineConfigNode::FAircraftEngineConfigNode(const UE::Dataflow::FNodeParameters& InParam, FGuid InGuid)
	: FAircraftConfigNodeBase(InParam, InGuid)
{
	if (FRichCurve* FullCurve = FullThrottleTorqueCurve.GetRichCurve())
	{
		FullCurve->Reset();
		FullCurve->AddKey(800.0f, 220.0f);
		FullCurve->AddKey(1500.0f, 320.0f);
		FullCurve->AddKey(2500.0f, 360.0f);
		FullCurve->AddKey(4000.0f, 340.0f);
		FullCurve->AddKey(5500.0f, 280.0f);
		FullCurve->AddKey(6500.0f, 0.0f);
	}

	if (FRichCurve* ZeroCurve = ZeroThrottleTorqueCurve.GetRichCurve())
	{
		ZeroCurve->Reset();
		ZeroCurve->AddKey(0.0f, 0.0f);
		ZeroCurve->AddKey(900.0f, 0.0f);
		ZeroCurve->AddKey(2500.0f, -30.0f);
		ZeroCurve->AddKey(4500.0f, -60.0f);
		ZeroCurve->AddKey(6500.0f, -90.0f);
	}

	RegisterAircraftConnections();
	RegisterInputConnection(&FullThrottleTorqueCurve);
	RegisterInputConnection(&ZeroThrottleTorqueCurve);
	RegisterInputConnection(&IdleRPM);
	RegisterInputConnection(&MaxRPM);
	RegisterInputConnection(&EngineInertia);
}

void FAircraftEngineConfigNode::AddProperties(FPropertyHelper& /*PropertyHelper*/) const
{
}

void FAircraftEngineConfigNode::EvaluateAircraftCollection(
	UE::Dataflow::FContext& /*Context*/,
	const TSharedRef<FManagedArrayCollection>& AircraftCollection,
	FAircraftConfigNodeBase::FAircraftFacade& InFacade) const
{
	using namespace UE::AircraftLab::AircraftAsset;

	if (AircraftCollection->NumElements(AircraftCollectionGroup::Powertrain) == 0)
	{
		AircraftCollection->AddElements(1, AircraftCollectionGroup::Powertrain);
	}
	else if (AircraftCollection->NumElements(AircraftCollectionGroup::Powertrain) > 1)
	{
		AircraftCollection->Resize(1, AircraftCollectionGroup::Powertrain);
	}

	InFacade.FindOrAddAttribute<FString>(
		AircraftCollectionAttribute::PowertrainEngineFullThrottleTorqueCurve,
		AircraftCollectionGroup::Powertrain)[0] =
		UE::AircraftLab::AircraftAssetDataflowNodes::Private::SerializeRuntimeFloatCurve(FullThrottleTorqueCurve);
	InFacade.FindOrAddAttribute<FString>(
		AircraftCollectionAttribute::PowertrainEngineZeroThrottleTorqueCurve,
		AircraftCollectionGroup::Powertrain)[0] =
		UE::AircraftLab::AircraftAssetDataflowNodes::Private::SerializeRuntimeFloatCurve(ZeroThrottleTorqueCurve);
	InFacade.FindOrAddAttribute<float>(
		AircraftCollectionAttribute::PowertrainEngineIdleRPM,
		AircraftCollectionGroup::Powertrain)[0] = IdleRPM;
	InFacade.FindOrAddAttribute<float>(
		AircraftCollectionAttribute::PowertrainEngineMaxRPM,
		AircraftCollectionGroup::Powertrain)[0] = MaxRPM;
	InFacade.FindOrAddAttribute<float>(
		AircraftCollectionAttribute::PowertrainEngineInertia,
		AircraftCollectionGroup::Powertrain)[0] = EngineInertia;
}

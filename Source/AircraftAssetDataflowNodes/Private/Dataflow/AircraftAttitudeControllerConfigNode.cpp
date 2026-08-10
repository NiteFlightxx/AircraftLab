#include "Dataflow/AircraftAttitudeControllerConfigNode.h"

#include "AircraftAsset/AircraftCollection.h"
#include "AircraftAsset/CollectionAircraftConstFacade.h"
#include "AircraftAsset/CollectionAircraftPropertyFacade.h"

#include "FlightControllerConfigNodeUtils.h"

#include UE_INLINE_GENERATED_CPP_BY_NAME(AircraftAttitudeControllerConfigNode)

FAircraftAttitudeControllerConfigNode::FAircraftAttitudeControllerConfigNode(const UE::Dataflow::FNodeParameters& InParam, FGuid InGuid)
	: FDataflowNode(InParam, InGuid)
{
	RegisterInputConnection(&Collection);
	RegisterOutputConnection(&Collection, &Collection);
}

void FAircraftAttitudeControllerConfigNode::Evaluate(UE::Dataflow::FContext& Context, const FDataflowOutput* Out) const
{
	using namespace UE::AircraftLab::AircraftAsset;
	using namespace UE::AircraftLab::AircraftAsset::Private;
	if (!Out || !Out->IsA<FManagedArrayCollection>(&Collection))
	{
		return;
	}
	if (Config.AngularDampingFeedForwardScale < 0.0f)
	{
		Context.Error(FText::FromString(TEXT("Attitude-controller angular damping must be non-negative.")), this);
	}

	const TSharedRef<FManagedArrayCollection> AircraftCollection = MakeShared<FManagedArrayCollection>(
		GetValue<FManagedArrayCollection>(Context, &Collection));
	FCollectionAircraftFacade Facade(AircraftCollection);
	Facade.DefineSchema();
#define UE_AIRCRAFT_WRITE_ATTITUDE(GetterName, Value) if (TArrayView<FVector3f> Values = Facade.Get##GetterName(); !Values.IsEmpty()) { Values[0] = Value; }
	UE_AIRCRAFT_WRITE_ATTITUDE(FcAngleKp, Config.QuaternionAttitudeGains)
	UE_AIRCRAFT_WRITE_ATTITUDE(FcAngleKi, FVector3f::ZeroVector)
	UE_AIRCRAFT_WRITE_ATTITUDE(FcAngleKd, FVector3f::ZeroVector)
	UE_AIRCRAFT_WRITE_ATTITUDE(FcRateKp, Config.RateKp)
	UE_AIRCRAFT_WRITE_ATTITUDE(FcRateKi, Config.RateKi)
	UE_AIRCRAFT_WRITE_ATTITUDE(FcRateKd, Config.RateKd)
#undef UE_AIRCRAFT_WRITE_ATTITUDE
	if (TArrayView<float> Values = Facade.GetFcDerivativeCutoffHz(); !Values.IsEmpty())
	{
		Values[0] = Config.RateDerivativeCutoffHz.Z;
	}

	FCollectionAircraftPropertyMutableFacade Properties(AircraftCollection);
	Properties.DefineSchema();
	SetConfigProperty(Properties, TEXT("FlightController.Attitude.RateIntegralLimit"), Config.RateIntegralLimit);
	SetConfigProperty(Properties, TEXT("FlightController.Attitude.RateOutputLimit"), Config.RateOutputLimit);
	SetConfigProperty(Properties, TEXT("FlightController.Attitude.RateDerivativeCutoffHz"), Config.RateDerivativeCutoffHz);
	SetConfigProperty(Properties, TEXT("FlightController.Attitude.AngularDampingFeedForwardScale"), Config.AngularDampingFeedForwardScale);
	SetConfigProperty(Properties, TEXT("FlightController.Attitude.EnableReferenceModel"), Config.bEnableAttitudeReferenceModel);
	SetConfigProperty(Properties, TEXT("FlightController.Attitude.ReferenceModelNaturalFrequency"), Config.ReferenceModelNaturalFrequency);
	SetConfigProperty(Properties, TEXT("FlightController.Attitude.ReferenceModelRateFeedForwardLimit"), Config.ReferenceModelRateFeedForwardLimitDegPerSec);
	SetValue(Context, MoveTemp(*AircraftCollection), &Collection);
}

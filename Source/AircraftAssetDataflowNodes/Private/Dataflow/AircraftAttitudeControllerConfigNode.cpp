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
	const FManagedArrayCollection InputCollection = GetValue<FManagedArrayCollection>(Context, &Collection);
	auto IsValidPid = [](const FAircraftFeedbackPidChannelConfig& Pid)
	{
		return FMath::IsFinite(Pid.Kp) && FMath::IsFinite(Pid.Ki) && FMath::IsFinite(Pid.Kd)
			&& FMath::IsFinite(Pid.IntegralLimit) && FMath::IsFinite(Pid.OutputLimit)
			&& FMath::IsFinite(Pid.DerivativeCutoffHz) && Pid.IntegralLimit >= 0.0f
			&& Pid.OutputLimit >= 0.0f && Pid.DerivativeCutoffHz >= 0.0f;
	};
	const bool bValidAttitudeGains = FMath::IsFinite(Config.QuaternionAttitudeGains.X)
		&& FMath::IsFinite(Config.QuaternionAttitudeGains.Y)
		&& FMath::IsFinite(Config.QuaternionAttitudeGains.Z)
		&& Config.QuaternionAttitudeGains.X >= 0.0f
		&& Config.QuaternionAttitudeGains.Y >= 0.0f
		&& Config.QuaternionAttitudeGains.Z >= 0.0f;
	if (!bValidAttitudeGains || !IsValidPid(Config.RollRate) || !IsValidPid(Config.PitchRate)
		|| !IsValidPid(Config.YawRate) || !FMath::IsFinite(Config.AngularDampingFeedForwardScale)
		|| !FMath::IsFinite(Config.ReferenceModelNaturalFrequency)
		|| !FMath::IsFinite(Config.ReferenceModelRateFeedForwardLimitDegPerSec)
		|| Config.AngularDampingFeedForwardScale < 0.0f
		|| Config.ReferenceModelNaturalFrequency < 0.5f
		|| Config.ReferenceModelRateFeedForwardLimitDegPerSec < 0.0f)
	{
		Context.Error(FText::FromString(TEXT("Attitude-controller PID or reference-model values are outside their valid range.")), this);
		SetValue(Context, InputCollection, &Collection);
		return;
	}

	const TSharedRef<FManagedArrayCollection> AircraftCollection = MakeShared<FManagedArrayCollection>(
		InputCollection);
	FCollectionAircraftFacade Facade(AircraftCollection);
	Facade.DefineSchema();
#define UE_AIRCRAFT_WRITE_ATTITUDE(GetterName, Value) if (TArrayView<FVector3f> Values = Facade.Get##GetterName(); !Values.IsEmpty()) { Values[0] = Value; }
	UE_AIRCRAFT_WRITE_ATTITUDE(FcAngleKp, Config.QuaternionAttitudeGains)
	UE_AIRCRAFT_WRITE_ATTITUDE(FcRateKp, FVector3f(Config.RollRate.Kp, Config.PitchRate.Kp, Config.YawRate.Kp))
	UE_AIRCRAFT_WRITE_ATTITUDE(FcRateKi, FVector3f(Config.RollRate.Ki, Config.PitchRate.Ki, Config.YawRate.Ki))
	UE_AIRCRAFT_WRITE_ATTITUDE(FcRateKd, FVector3f(Config.RollRate.Kd, Config.PitchRate.Kd, Config.YawRate.Kd))
#undef UE_AIRCRAFT_WRITE_ATTITUDE

	FCollectionAircraftPropertyMutableFacade Properties(AircraftCollection);
	Properties.DefineSchema();
	SetConfigProperty(Properties, TEXT("FlightController.Attitude.RateIntegralLimit"), FVector3f(Config.RollRate.IntegralLimit, Config.PitchRate.IntegralLimit, Config.YawRate.IntegralLimit));
	SetConfigProperty(Properties, TEXT("FlightController.Attitude.RateOutputLimit"), FVector3f(Config.RollRate.OutputLimit, Config.PitchRate.OutputLimit, Config.YawRate.OutputLimit));
	SetConfigProperty(Properties, TEXT("FlightController.Attitude.RateDerivativeCutoffHz"), FVector3f(Config.RollRate.DerivativeCutoffHz, Config.PitchRate.DerivativeCutoffHz, Config.YawRate.DerivativeCutoffHz));
	SetConfigProperty(Properties, TEXT("FlightController.Attitude.RollRateFreezeIntegralWhenSaturated"), Config.RollRate.bFreezeIntegralWhenSaturated);
	SetConfigProperty(Properties, TEXT("FlightController.Attitude.PitchRateFreezeIntegralWhenSaturated"), Config.PitchRate.bFreezeIntegralWhenSaturated);
	SetConfigProperty(Properties, TEXT("FlightController.Attitude.YawRateFreezeIntegralWhenSaturated"), Config.YawRate.bFreezeIntegralWhenSaturated);
	SetConfigProperty(Properties, TEXT("FlightController.Attitude.AngularDampingFeedForwardScale"), Config.AngularDampingFeedForwardScale);
	SetConfigProperty(Properties, TEXT("FlightController.Attitude.EnableReferenceModel"), Config.bEnableAttitudeReferenceModel);
	SetConfigProperty(Properties, TEXT("FlightController.Attitude.ReferenceModelNaturalFrequency"), Config.ReferenceModelNaturalFrequency);
	SetConfigProperty(Properties, TEXT("FlightController.Attitude.ReferenceModelRateFeedForwardLimit"), Config.ReferenceModelRateFeedForwardLimitDegPerSec);
	SetValue(Context, MoveTemp(*AircraftCollection), &Collection);
}

#include "Dataflow/AircraftThrottleInputConfigNode.h"

#include UE_INLINE_GENERATED_CPP_BY_NAME(AircraftThrottleInputConfigNode)

FAircraftThrottleInputConfigNode::FAircraftThrottleInputConfigNode(
	const UE::Dataflow::FNodeParameters& InParam,
	FGuid InGuid)
	: FAircraftConfigNodeBase(InParam, InGuid)
{
	RegisterAircraftConnections();
	RegisterInputConnection(&RiseRate);
	RegisterInputConnection(&FallRate);
	RegisterInputConnection(&bUseRiseRateCurve);
	RegisterInputConnection(&RiseRateCurve);
	RegisterInputConnection(&bUseFallRateCurve);
	RegisterInputConnection(&FallRateCurve);
}

void FAircraftThrottleInputConfigNode::AddProperties(FPropertyHelper& /*PropertyHelper*/) const
{
}

void FAircraftThrottleInputConfigNode::EvaluateAircraftCollection(
	UE::Dataflow::FContext& /*Context*/,
	const TSharedRef<FManagedArrayCollection>& /*AircraftCollection*/,
	FAircraftConfigNodeBase::FAircraftFacade& /*InFacade*/) const
{
}



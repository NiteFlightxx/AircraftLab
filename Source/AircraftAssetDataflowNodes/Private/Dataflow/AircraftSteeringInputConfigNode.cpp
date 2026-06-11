#include "Dataflow/AircraftSteeringInputConfigNode.h"


#include UE_INLINE_GENERATED_CPP_BY_NAME(AircraftSteeringInputConfigNode)

FAircraftSteeringInputConfigNode::FAircraftSteeringInputConfigNode(
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

void FAircraftSteeringInputConfigNode::AddProperties(FPropertyHelper& /*PropertyHelper*/) const
{
}

void FAircraftSteeringInputConfigNode::EvaluateAircraftCollection(
	UE::Dataflow::FContext& /*Context*/,
	const TSharedRef<FManagedArrayCollection>& /*AircraftCollection*/) const
{
}

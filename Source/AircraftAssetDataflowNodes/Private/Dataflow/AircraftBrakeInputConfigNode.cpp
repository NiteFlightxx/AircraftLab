#include "Dataflow/AircraftBrakeInputConfigNode.h"



#include UE_INLINE_GENERATED_CPP_BY_NAME(AircraftBrakeInputConfigNode)

FAircraftBrakeInputConfigNode::FAircraftBrakeInputConfigNode(
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

void FAircraftBrakeInputConfigNode::AddProperties(FPropertyHelper& /*PropertyHelper*/) const
{
}

void FAircraftBrakeInputConfigNode::EvaluateAircraftCollection(
	UE::Dataflow::FContext& /*Context*/,
	const TSharedRef<FManagedArrayCollection>& /*AircraftCollection*/) const
{
}

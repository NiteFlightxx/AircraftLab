#include "Dataflow/AircraftHandbrakeInputConfigNode.h"


#include UE_INLINE_GENERATED_CPP_BY_NAME(AircraftHandbrakeInputConfigNode)

FAircraftHandbrakeInputConfigNode::FAircraftHandbrakeInputConfigNode(
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

void FAircraftHandbrakeInputConfigNode::AddProperties(FPropertyHelper& /*PropertyHelper*/) const
{
}

void FAircraftHandbrakeInputConfigNode::EvaluateAircraftCollection(
	UE::Dataflow::FContext& /*Context*/,
	const TSharedRef<FManagedArrayCollection>& /*AircraftCollection*/,
	FAircraftConfigNodeBase::FAircraftFacade& /*InFacade*/) const
{
}


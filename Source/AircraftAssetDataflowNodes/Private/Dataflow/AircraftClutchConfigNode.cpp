#include "Dataflow/AircraftClutchConfigNode.h"


#include UE_INLINE_GENERATED_CPP_BY_NAME(AircraftClutchConfigNode)

FAircraftClutchConfigNode::FAircraftClutchConfigNode(
	const UE::Dataflow::FNodeParameters& InParam,
	FGuid InGuid)
	: FAircraftConfigNodeBase(InParam, InGuid)
{
	RegisterAircraftConnections();
	RegisterInputConnection(&CapacityNm);
	RegisterInputConnection(&StiffnessNmPerRadPerSec);
}

void FAircraftClutchConfigNode::AddProperties(FPropertyHelper& /*PropertyHelper*/) const
{
}

void FAircraftClutchConfigNode::EvaluateAircraftCollection(
	UE::Dataflow::FContext& /*Context*/,
	const TSharedRef<FManagedArrayCollection>& /*AircraftCollection*/) const
{
}

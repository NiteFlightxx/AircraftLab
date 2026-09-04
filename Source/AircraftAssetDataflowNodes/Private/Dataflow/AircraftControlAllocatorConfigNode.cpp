#include "Dataflow/AircraftControlAllocatorConfigNode.h"

#include "FlightControllerConfigNodeUtils.h"

#include UE_INLINE_GENERATED_CPP_BY_NAME(AircraftControlAllocatorConfigNode)

FAircraftControlAllocatorConfigNode::FAircraftControlAllocatorConfigNode(const UE::Dataflow::FNodeParameters& InParam, FGuid InGuid)
	: FAircraftConfigNodeBase(InParam, InGuid)
{
	RegisterAircraftConnections();
}

bool FAircraftControlAllocatorConfigNode::ApplyToAircraftCollection(FAircraftConfigEvaluationContext& Context) const
{
	using namespace UE::AircraftLab::AircraftAsset::Private;
	if (!FMath::IsFinite(Config.DampedPseudoInverseLambda) || !FMath::IsFinite(Config.MinimumCosTilt)
		|| Config.DampedPseudoInverseLambda < 0.0f || Config.MinimumCosTilt < 0.05f || Config.MinimumCosTilt > 1.0f)
	{
		return Context.Error(TEXT("Control-allocator values are outside their valid range."));
	}

	auto& Facade = Context.GetAircraft();
	if (TArrayView<float> Values = Facade.GetFcAllocationDamping(); !Values.IsEmpty())
	{
		Values[0] = Config.DampedPseudoInverseLambda;
	}

	auto& Properties = Context.GetProperties();
	SetConfigProperty(Properties, TEXT("FlightController.Allocator.EnableTiltCompensation"), Config.bEnableTiltCompensation);
	SetConfigProperty(Properties, TEXT("FlightController.Allocator.MinimumCosTilt"), Config.MinimumCosTilt);
	return true;
}

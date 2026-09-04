#include "Dataflow/AircraftAerodynamicsConfigNode.h"

#include "Dataflow/FlightControllerConfigNodeUtils.h"

#include UE_INLINE_GENERATED_CPP_BY_NAME(AircraftAerodynamicsConfigNode)

FAircraftAerodynamicsConfigNode::FAircraftAerodynamicsConfigNode(
	const UE::Dataflow::FNodeParameters& InParam, FGuid InGuid)
	: FAircraftConfigNodeBase(InParam, InGuid)
{
	RegisterAircraftConnections();
}

bool FAircraftAerodynamicsConfigNode::ApplyToAircraftCollection(
	FAircraftConfigEvaluationContext& Context) const
{
	using namespace UE::AircraftLab::AircraftAsset::Private;
	auto IsNonNegativeVector = [](const FVector& Value)
	{
		return !Value.ContainsNaN() && Value.GetMin() >= 0.0;
	};
	const bool bInvalid = !FMath::IsFinite(Config.AirDensityKgPerM3)
		|| Config.AirDensityKgPerM3 <= 0.0f
		|| !IsNonNegativeVector(Config.LinearDragNsPerM)
		|| !IsNonNegativeVector(Config.DragAreaCoefficientM2)
		|| !IsNonNegativeVector(Config.AngularDragNmPerRadPerSec)
		|| !IsNonNegativeVector(Config.QuadraticAngularDragNmPerRadPerSecSq)
		|| !FMath::IsFinite(Config.MaxRelativeAirspeedCmPerSec)
		|| Config.MaxRelativeAirspeedCmPerSec <= 0.0f;
	if (bInvalid)
	{
		return Context.Error(TEXT("Aerodynamics configuration is invalid."));
	}

	auto& Properties = Context.GetProperties();
	SetConfigProperty(Properties, TEXT("Aerodynamics.Configured"), true);
	SetConfigProperty(Properties, TEXT("Aerodynamics.AirDensityKgPerM3"), Config.AirDensityKgPerM3);
	SetConfigProperty(Properties, TEXT("Aerodynamics.LinearDragNsPerM"), FVector3f(Config.LinearDragNsPerM));
	SetConfigProperty(Properties, TEXT("Aerodynamics.DragAreaCoefficientM2"), FVector3f(Config.DragAreaCoefficientM2));
	SetConfigProperty(Properties, TEXT("Aerodynamics.AngularDragNmPerRadPerSec"), FVector3f(Config.AngularDragNmPerRadPerSec));
	SetConfigProperty(Properties, TEXT("Aerodynamics.QuadraticAngularDragNmPerRadPerSecSq"), FVector3f(Config.QuadraticAngularDragNmPerRadPerSecSq));
	SetConfigProperty(Properties, TEXT("Aerodynamics.MaxRelativeAirspeedCmPerSec"), Config.MaxRelativeAirspeedCmPerSec);
	return true;
}

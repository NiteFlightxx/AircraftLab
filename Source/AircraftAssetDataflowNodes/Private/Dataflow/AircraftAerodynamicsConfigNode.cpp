#include "Dataflow/AircraftAerodynamicsConfigNode.h"

#include "AircraftAsset/CollectionAircraftPropertyFacade.h"
#include "Dataflow/FlightControllerConfigNodeUtils.h"

#include UE_INLINE_GENERATED_CPP_BY_NAME(AircraftAerodynamicsConfigNode)

FAircraftAerodynamicsConfigNode::FAircraftAerodynamicsConfigNode(
	const UE::Dataflow::FNodeParameters& InParam, FGuid InGuid)
	: FDataflowNode(InParam, InGuid)
{
	RegisterInputConnection(&Collection);
	RegisterOutputConnection(&Collection, &Collection);
}

void FAircraftAerodynamicsConfigNode::Evaluate(
	UE::Dataflow::FContext& Context, const FDataflowOutput* Out) const
{
	using namespace UE::AircraftLab::AircraftAsset;
	using namespace UE::AircraftLab::AircraftAsset::Private;
	if (!Out || !Out->IsA<FManagedArrayCollection>(&Collection))
	{
		return;
	}

	const FManagedArrayCollection Input = GetValue<FManagedArrayCollection>(Context, &Collection);
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
		Context.Error(FText::FromString(TEXT("Aerodynamics configuration is invalid.")), this);
		SetValue(Context, Input, &Collection);
		return;
	}

	const TSharedRef<FManagedArrayCollection> Output = MakeShared<FManagedArrayCollection>(Input);
	FCollectionAircraftPropertyMutableFacade Properties(Output);
	Properties.DefineSchema();
	SetConfigProperty(Properties, TEXT("Aerodynamics.Configured"), true);
	SetConfigProperty(Properties, TEXT("Aerodynamics.AirDensityKgPerM3"), Config.AirDensityKgPerM3);
	SetConfigProperty(Properties, TEXT("Aerodynamics.LinearDragNsPerM"), FVector3f(Config.LinearDragNsPerM));
	SetConfigProperty(Properties, TEXT("Aerodynamics.DragAreaCoefficientM2"), FVector3f(Config.DragAreaCoefficientM2));
	SetConfigProperty(Properties, TEXT("Aerodynamics.AngularDragNmPerRadPerSec"), FVector3f(Config.AngularDragNmPerRadPerSec));
	SetConfigProperty(Properties, TEXT("Aerodynamics.QuadraticAngularDragNmPerRadPerSecSq"), FVector3f(Config.QuadraticAngularDragNmPerRadPerSecSq));
	SetConfigProperty(Properties, TEXT("Aerodynamics.MaxRelativeAirspeedCmPerSec"), Config.MaxRelativeAirspeedCmPerSec);
	SetValue(Context, MoveTemp(*Output), &Collection);
}
